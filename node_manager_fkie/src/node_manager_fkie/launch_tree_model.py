# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from python_qt_binding.QtCore import QFile, Qt, Signal
from python_qt_binding.QtGui import QIcon, QStandardItem, QStandardItemModel
import os
import re
import roslib
import rospy
import traceback
from urlparse import urlparse

from master_discovery_fkie.common import get_hostname, subdomain
from master_discovery_fkie.master_info import NodeInfo
from node_manager_fkie.common import utf8
from node_manager_fkie.name_resolution import NameResolution
from parameter_handler import ParameterHandler
import node_manager_fkie as nm
from node_manager_daemon_fkie.file_item import FileItem


# ###############################################################################
# #############                  FileItem                          ##############
# ###############################################################################
class PathItem(QStandardItem):
    '''
    The PathItem stores the information about a file/directory.
    '''
    ITEM_TYPE = Qt.UserRole + 41

    NOT_FOUND = -1
    UNKNOWN = 0
    PROFILE = 5
    LAUNCH_FILE = 11
    CFG_FILE = 12
    FOLDER = 20
    PACKAGE = 21
    STACK = 22
    REMOTE_DAEMON = 23

    def __init__(self, path, parent=None, path_id=0):
        '''
        Initialize the PathItem object with given values. The name of the item is the base name of the path.
        Examples of paths:
            grpc://localhost:12311:                 -> name: @localhost
            grpc://localhost:12311:/absolute/path   -> name: path
            grpc://localhost:12311:relative/path    -> name: path
            grpc://localhost::with/default/port     -> name: port
            /absolute/local/path                    -> name: path
            relative/local/path                     -> name: path
        :param str path: path of the file
        :param parent: the parent path item.
        :type parent: QtGui.QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>
        '''
        self.url_parse_result = urlparse(path)
        self._path = path
        self._name = ''
        if not self.url_parse_result.path:
            self._name = '@%s' + self.url_parse_result.hostname
        else:
            self._name = os.path.basename(self.url_parse_result.path)
        QStandardItem.__init__(self, self._name)
        self.parent_item = parent
        self.id = path_id

    def _identify_path_on_ext(self, url_parse_result):
        '''
        Determines the id based on file extension.
        :param url_parse_result: already parsed path.
        :type url_parse_result: urlparse.ParseResult
        :return: the id represents whether it is a file, package or stack
        :rtype: constants of LaunchItem
        '''
        if self.id != 0:
            return self._id
        path = url_parse_result.path
        _filename, file_extension = os.path.splitext(path)
        if not file_extension:
            return PathItem.UNKNOWN
        if file_extension == '.launch':
            return PathItem.LAUNCH_FILE
        elif file_extension == '.nmprofile':
            return PathItem.PROFILE
        elif file_extension in nm.settings().launch_view_file_ext:
            return PathItem.CFG_FILE
        return PathItem.UNKNOWN

    def _update_icon(self):
        if self.id == PathItem.UNKNOWN:
            return
        if self.id == PathItem.FOLDER:
            self.setIcon(QIcon(":/icons/crystal_clear_folder.png"))
        elif self.id == PathItem.PACKAGE:
            self.setIcon(QIcon(":/icons/crystal_clear_package.png"))
        elif self.id == PathItem.LAUNCH_FILE:
            if self.parent_item is None:
                self.setIcon(QIcon(":/icons/crystal_clear_launch_file_recent.png"))
            else:
                self.setIcon(QIcon(":/icons/crystal_clear_launch_file.png"))
        elif self.id == PathItem.STACK:
            self.setIcon(QIcon(":/icons/crystal_clear_stack.png"))
        elif self.id == PathItem.PROFILE:
            if self.parent_item is None:
                self.setIcon(QIcon(":/icons/crystal_clear_profile_recent.png"))
            else:
                self.setIcon(QIcon(":/icons/crystal_clear_profile.png"))
        elif self.id == PathItem.REMOTE_DAEMON:
            self.setIcon(QIcon(":/icons/stock_connect.png"))

    @property
    def name(self):
        '''
        The name of this group.
        :rtype: str
        '''
        return self._name

    @name.setter
    def name(self, new_name):
        '''
        Set the new name of this path and updates the displayed name of the item.
        :param str new_name: new name
        '''
        self._name = new_name
        self.setText(self._name)

    def addNode(self, node, cfg=''):
        '''
        Adds a new node with given name.
        @param node: the NodeInfo of the node to create
        @type node: L{NodeInfo}
        @param cfg: The configuration, which describes the node
        @type cfg: C{str}
        '''
        groups = self.getCapabilityGroups(node.name)
        if groups:
            for _, group_list in groups.items():
                for group_name in group_list:
                    # insert in the group
                    groupItem = self.getGroupItem(group_name)
                    groupItem.addNode(node, cfg)
        else:
            # insert in order
            new_item_row = NodeItem.newNodeRow(node.name, node.masteruri)
            self._addRow_sorted(new_item_row)
            new_item_row[0].node_info = node
            if cfg or cfg == '':
                new_item_row[0].addConfig(cfg)

    def _addRow_sorted(self, row):
        for i in range(self.rowCount()):
            item = self.child(i)
            if item > row[0].name:
                self.insertRow(i, row)
                row[0].parent_item = self
                return
        self.appendRow(row)
        row[0].parent_item = self

    def clearUp(self, fixed_node_names=None):
        '''
        Removes not running and not configured nodes.
        @param fixed_node_names: If the list is not None, the node not in the list are
        set to not running!
        @type fixed_node_names: C{[str]}
        '''
        # first clear sub groups
        groups = self.getGroupItems()
        for group in groups:
            group.clearUp(fixed_node_names)
        removed = False
        # move running nodes without configuration to the upper layer, remove not running and duplicate nodes
        for i in reversed(range(self.rowCount())):
            item = self.child(i)
            if isinstance(item, NodeItem):
                # set the running state of the node to None
                if fixed_node_names is not None and item.name not in fixed_node_names:
                    item.node_info = NodeInfo(item.name, item.node_info.masteruri)
                if not (item.has_configs() or item.is_running() or item.published or item.subscribed or item.services):
                    removed = True
                    self.removeRow(i)
                elif not isinstance(self, HostItem):
                    has_launches = NodeItem.has_launch_cfgs(item.cfgs)
                    has_defaults = NodeItem.has_default_cfgs(item.cfgs)
                    has_std_cfg = item.has_std_cfg()
                    if item.state == NodeItem.STATE_RUN and not (has_launches or has_defaults or has_std_cfg):
                        # if it is in a group, is running, but has no configuration, move it to the host
                        if self.parent_item is not None and isinstance(self.parent_item, HostItem):
                            items_in_host = self.parent_item.getNodeItemsByName(item.name, True)
                            if len(items_in_host) == 1:
                                row = self.takeRow(i)
                                self.parent_item._addRow_sorted(row)
                            else:
                                # remove item
                                removed = True
                                self.removeRow(i)
        if removed:
            self.updateIcon()

        # remove empty groups
        for i in reversed(range(self.rowCount())):
            item = self.child(i)
            if isinstance(item, PathItem):
                if item.rowCount() == 0:
                    self.removeRow(i)

    def __eq__(self, item):
        '''
        Compares the name of the item.
        '''
        if isinstance(item, str) or isinstance(item, unicode):
            return self.name.lower() == item.lower()
        elif not (item is None):
            return self.name.lower() == item.name.lower()
        return False

    def __gt__(self, item):
        '''
        Compares the name of the item.
        '''
        if isinstance(item, str) or isinstance(item, unicode):
            return self.name.lower() > item.lower()
        elif not (item is None):
            return self.name.lower() > item.name.lower()
        return False


# ###############################################################################
# #############                LanchTreeModel                      ##############
# ###############################################################################

class LaunchTreeModel(QStandardItemModel):
    '''
    The model to show file tree on local and remote hosts.
    '''

    header = [('Name', 450),
              ('Date Modified', 80),
              ('Size', -1)]

    def __init__(self, parent=None):
        '''
        Initialize the model.
        '''
        super(LaunchTreeModel, self).__init__(parent)
        self.setColumnCount(len(LaunchTreeModel.header))
        self.setHorizontalHeaderLabels([label for label, _ in LaunchTreeModel.header])

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def get_hostitem(self, masteruri, address):
        '''
        Searches for the host item in the model. If no item is found a new one will
        created and inserted in sorted order.
        @param masteruri: ROS master URI
        @type masteruri: C{str}
        @param address: the address of the host
        @type address: C{str}
        @return: the item associated with the given master
        @rtype: L{HostItem}
        '''
        if masteruri is None:
            return None
        resaddr = nm.nameres().hostname(address)
        host = (masteruri, resaddr)
        # [address] + nm.nameres().resolve_cached(address)
        local = (self.local_addr in [address] + nm.nameres().resolve_cached(address) and
                 self._local_masteruri == masteruri)
        # find the host item by address
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            if root.child(i) == host:
                return root.child(i)
            elif root.child(i) > host:
                hostItem = HostItem(masteruri, resaddr, local)
                self.insertRow(i, hostItem)
                self.hostInserted.emit(hostItem)
                self._set_std_capabilities(hostItem)
                return hostItem
        hostItem = HostItem(masteruri, resaddr, local)
        self.appendRow(hostItem)
        self.hostInserted.emit(hostItem)
        self._set_std_capabilities(hostItem)
        return hostItem

    def get_file_item(self, path):
        '''
        :return: Return an existing ``FileItem`` object corresponding to the given path.
        :rtype: FileItem
        :raise: LookupError if no item for given path was found
        '''
        if not path:
            return self.invisibleRootItem()
        p_res = urlparse(path)
        if p_res.scheme == 'grpc':
            if p_res.hostname is None:
                raise LookupError("can not find FileItem for %s. Invalid hostname in `grpc` scheme!" % path)
            root = self.invisibleRootItem()
            for i in range(root.rowCount()):
                if root.child(i) == "@%s" % p_res.hostname:
                    

    def update_model(self, root_path, file_items):
        '''
        Updates the model data.
        :param str root_path: the root path containing the file items.
        :param paths: a list with name and info objects of the nodes.
        :type paths: [node_manager_daemon_fkie.file_item.FileItem]
        '''
        for item in file_items:
            if item.type = FileItem.DIR:
                
        for (name, node) in nodes.items():
            addr = get_hostname(node.uri if node.uri is not None else node.masteruri)
            addresses.append(node.masteruri)
            host = (node.masteruri, addr)
            if host not in hosts:
                hosts[host] = dict()
            hosts[host][name] = node
        # update nodes for each host
        for ((masteruri, host), nodes_filtered) in hosts.items():
            hostItem = self.get_hostitem(masteruri, host)
            # rename the host item if needed
            if hostItem is not None:
                hostItem.updateRunningNodeState(nodes_filtered)
            # request for all nodes in host the parameter capability_group
            self._requestCapabilityGroupParameter(hostItem)
        # update nodes of the hosts, which are not more exists
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            if host.masteruri not in addresses:
                host.updateRunningNodeState({})
        self.removeEmptyHosts()
        # update the duplicate state
#    self.markNodesAsDuplicateOf(self.getRunningNodes())
