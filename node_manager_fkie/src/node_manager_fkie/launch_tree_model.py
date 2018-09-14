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
import grpc
import os
import re
import roslib
import rospy
import uuid
import traceback
from urlparse import urlparse

import node_manager_fkie as nm
from master_discovery_fkie.common import get_hostname, subdomain
from master_discovery_fkie.master_info import NodeInfo
from node_manager_daemon_fkie.file_item import FileItem
from .common import utf8
from .name_resolution import NameResolution
from .parameter_handler import ParameterHandler
from .detailed_msg_box import MessageBox
from .nmd_client import NmdClient


# ###############################################################################
# #############                  FileItem                          ##############
# ###############################################################################
class PathItem(QStandardItem):
    '''
    The PathItem stores the information about a file/directory.
    '''
    ITEM_TYPE = Qt.UserRole + 41
    NAME_ROLE = Qt.UserRole + 1
    DATE_ROLE = Qt.UserRole + 2
    SIZE_ROLE = Qt.UserRole + 3

    NOT_FOUND = -1
    UNKNOWN = 0
    PROFILE = 5
    RECENT_PROFILE = 6
    RECENT_FILE = 10
    LAUNCH_FILE = 11
    CFG_FILE = 12
    FOLDER = 20
    PACKAGE = 21
    STACK = 22
    REMOTE_DAEMON = 23

    def __init__(self, path, path_id, mtime, size, parent=None):
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
        self.mtime = mtime
        self.size = size
        self.id = path_id
        self.parent_item = parent
        self.item_mtime = QStandardItem()
        self.item_mtime.setText(utf8(mtime))
        self.item_size = QStandardItem()
        self.item_size.setText(utf8(size))  # TODO: add size unit

    def _identify_path_on_ext(self, url_parse_result):
        '''
        Determines the id based on file extension.
        :param url_parse_result: already parsed path.
        :type url_parse_result: urlparse.ParseResult
        :return: the id represents whether it is a file, package or stack
        :rtype: constants of PathItem
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

    def type(self):
        return PathItem.ITEM_TYPE

    def data(self, role):
        '''
        The view asks us for all sorts of information about our data...
        :param role: the art of the data
        :type role: U{QtCore.Qt.DisplayRole<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        :see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if role == Qt.DisplayRole:
            # return the displayed item name
            if self.id == PathItem.RECENT_FILE or self.id == PathItem.RECENT_PROFILE:
                return "%s   [%s]" % (self.name, self.package_name)  # .decode(sys.getfilesystemencoding())
            elif self.id == PathItem.REMOTE_DAEMON:
                return "//%s" % self.name
            else:
                return "%s" % self.name
        elif role == Qt.ToolTipRole:
            # return the tooltip of the item
            result = "%s" % self.path
            if self.id == PathItem.RECENT_FILE or self.id == PathItem.RECENT_PROFILE:
                result = "%s\nPress 'Delete' to remove the entry from the history list" % self.path
            return result
#     elif role == Qt.DecorationRole:
#       # return the showed icon
#       pathItem, path, pathId = self.items[index.row()]
#       if self.id > LaunchListModel.NOTHING and self.model().icons.has_key(self.id):
#         return self.model().icons[self.id]
#       return None
        elif role == Qt.EditRole:
            return "%s" % self.name
        else:
            # We don't care about anything else, so return default value
            return QStandardItem.data(self, role)

    def setData(self, value, role=Qt.EditRole):
        # TODO: fix to use it with nm_daemon
        if role == Qt.EditRole:
            # rename the file or folder
            if self.name != value and self.id in [self.RECENT_FILE, self.LAUNCH_FILE, self.RECENT_PROFILE, self.PROFILE, self.CFG_FILE, self.FOLDER]:
                new_path = os.path.join(os.path.dirname(self.path), value)
                if not os.path.exists(new_path):
                    os.rename(self.path, new_path)
                    if self.name != value and self.id in [self.RECENT_FILE, self.RECENT_PROFILE]:
                        # update in history
                        nm.settings().launch_history_add(new_path, replace=self.path)
                    self.name = value
                    self.path = new_path
                else:
                    MessageBox.warning(self, "Path already exists",
                                       "`%s` already exists!" % value, "Complete path: %s" % new_path)
        return QStandardItem.setData(self, value, role)

    def isLaunchFile(self):
        '''
        :return: True if it is a launch file
        :rtype: bool
        '''
        return self.path is not None and os.path.isfile(self.path) and self.path.endswith('.launch')

    def isConfigFile(self):
        '''
        :return: True if it is a config file
        :rtype: bool
        '''
        return self.id == self.CFG_FILE

    def is_profile_file(self):
        '''
        :return: True if it is a node_manager profile file
        :rtype: bool
        '''
        return self.path is not None and os.path.isfile(self.path) and self.path.endswith('.nmprofile')

    @classmethod
    def create_row_items(self, path, path_id, mtime, size, root):
        '''
        Creates the list of the items. This list is used for the
        visualization of path data as a table row.
        :param str path: file path
        :param int path_id: identification of the path (folder, package, launch file, ...)
        :param int mtime: modification time
        :param int size: file size
        :param root: The parent QStandardItem
        :type root: U{QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}
        :return: the list for the representation as a row
        :rtype: C{[L{PathItem} or U{QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}, ...]}
        '''
        items = []
        item = PathItem(path, path_id, mtime, size, parent=root)
        items.append(item)
        items.append(item.item_mtime)
        items.append(item.item_size)
        return items

    def __eq__(self, item):
        '''
        Compares the path of the item.
        '''
        if isinstance(item, str) or isinstance(item, unicode):
            return self.path.lower() == item.lower()
        elif not (item is None):
            return self.path.lower() == item.path.lower()
        return False

    def __gt__(self, item):
        '''
        Compares the path of the item.
        '''
        if isinstance(item, str) or isinstance(item, unicode):
            return self.path.lower() > item.lower()
        elif not (item is None):
            return self.path.lower() > item.path.lower()
        return False


# ###############################################################################
# #############                LanchTreeModel                      ##############
# ###############################################################################

class LaunchTreeModel(QStandardItemModel):
    '''
    The model to show file tree on local and remote hosts.
    '''

    header = [('Name', 350),
              ('Date Modified', 80),
              ('Size', -1)]

    def __init__(self, parent=None, progress_queue=None):
        '''
        Initialize the model.
        '''
        QStandardItemModel.__init__(self, parent)
        self.setColumnCount(len(LaunchTreeModel.header))
        self.setHorizontalHeaderLabels([label for label, _ in LaunchTreeModel.header])
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
        self._progress_queue = progress_queue
        self._nmd_client = NmdClient()
        self._nmd_client.listed_path.connect(self._listed_path)
        self._nmd_client.error.connect(self._nmd_error)
        self._current_path = 'grpc://localhost:12321'
        self._nmd_client.list_path_threaded("", self._current_path)
        self.count = 0

    def _getRootItems(self):
        result = list(nm.settings().launch_history)
        result.extend(self.root_paths)
        return result

    def _listed_path(self, url, path, result):
        print "RESULT", path, result
        result_list = []
        for path_item in result:
            item = os.path.normpath(''.join([path, '/', path_item.path]))
            fullpath = os.path.basename(item)
            if fullpath == 'src':
                fullpath = '%s (src)' % os.path.basename(os.path.dirname(item))
            path_id = PathItem.NOT_FOUND
            if FileItem.FILE == 0:
                path_id = PathItem.CFG_FILE
                # TODO
            elif FileItem.DIR == 1:
                path_id = PathItem.FOLDER
            elif FileItem.SYMLINK == 2:
                pass
            elif FileItem.PACKAGE == 3:
                path_id = PathItem.PACKAGE
            if (path_id != PathItem.NOT_FOUND):
                new_item_row = PathItem.create_row_items(path_item.path, path_id, path_item.mtime, path_item.size, self.get_file_item(path))
                self.get_file_item(path).appendRow(new_item_row)
                self.pyqt_workaround[path_item.path] = new_item_row[0]
                # new_item_row[0].updateView()

#                result_list.append((fullpath, item, path_id))
#        self._setNewList((path, result_list))

    def _nmd_error(self, method, url, path, error):
        if self.count > 0:
            import time
            time.sleep(2)
            self._nmd_client.list_path_threaded("", self._current_path)
            return
        if hasattr(error, "code") and error.code() == grpc.StatusCode.UNAVAILABLE:
            name = "node_manager_daemon"
            if self._progress_queue is not None:
                rospy.loginfo("Connection problem to %s, try to start <%s>" % (url, name))
                self._progress_queue.add2queue(utf8(uuid.uuid4()),
                                               'start %s' % name,
                                               nm.starter().runNodeWithoutConfig,
                                               (get_hostname(url), '%s_fkie' % name, name,
                                                nm.nameres().normalize_name(name), [],
                                                None, False))
                self._progress_queue.start()
                self.count = 1
                self._nmd_client.list_path_threaded("", self._current_path)
            else:
                rospy.logwarn("Error while call <%s> on '%s': %s" % (method, url, error))
                rospy.logwarn("can't start <%s>, no progress queue defined!" % name)

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

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
                    pass  # TODO
        return self.invisibleRootItem()

    def update_model(self, root_path, file_items):
        '''
        Updates the model data.
        :param str root_path: the root path containing the file items.
        :param paths: a list with name and info objects of the nodes.
        :type paths: [node_manager_daemon_fkie.file_item.FileItem]
        '''
        for item in file_items:
            if item.type == FileItem.DIR:
                pass
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

    def show_packages(self, show):
        pass

