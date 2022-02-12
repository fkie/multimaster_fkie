# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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



from python_qt_binding.QtCore import QFile, QRect, Qt, Signal
from python_qt_binding.QtGui import QIcon, QImage, QStandardItem, QStandardItemModel
try:
    from python_qt_binding.QtGui import QItemDelegate
except Exception:
    from python_qt_binding.QtWidgets import QItemDelegate
from datetime import datetime
import hashlib
import re
import roslib
import rospy
import traceback

from diagnostic_msgs.msg import KeyValue
from fkie_master_discovery.common import get_hostname, subdomain
from fkie_master_discovery.master_info import NodeInfo
from fkie_node_manager_daemon.common import sizeof_fmt, isstring, utf8
from fkie_node_manager_daemon import url as nmdurl
from fkie_node_manager.common import lnamespace, namespace, normns
from fkie_node_manager.name_resolution import NameResolution, MasterEntry
from fkie_node_manager.parameter_handler import ParameterHandler
import fkie_node_manager as nm


class CellItem(QStandardItem):
    '''
    Item for a cell. References to a node item.
    '''
    ITEM_TYPE = Qt.UserRole + 41

    def __init__(self, name, item=None, parent=None):
        '''
        Initialize the CellItem object with given values.

        :param name: the name of the group
        :param parent: the parent item. In most cases this is the HostItem. The variable is used to determine the different columns of the NodeItem.
        :type parent: :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>
        '''
        QStandardItem.__init__(self)
        self.parent_item = parent
        self._name = name
        self.item = item

    @property
    def name(self):
        '''
        The name of this group.

        :rtype: str
        '''
        return self._name


# ###############################################################################
# #############                  GrouptItem                        ##############
# ###############################################################################
class GroupItem(QStandardItem):
    '''
    The GroupItem stores the information about a group of nodes.
    '''
    ITEM_TYPE = Qt.UserRole + 25

    def __init__(self, name, parent=None, has_remote_launched_nodes=False, is_group=False):
        '''
        Initialize the GroupItem object with given values.

        :param str name: the name of the group
        :param parent: the parent item. In most cases this is the HostItem. The variable is used to determine the different columns of the NodeItem.
        :type parent: :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>
        :param bool is_group: True if this is a capability group. In other case it is a namespace group.
        '''
        dname = name
        if dname.rfind('@') <= 0:
            if is_group:
                dname = '{' + dname + '}'
            else:
                dname = dname + '/'
        QStandardItem.__init__(self, dname)
        self.parent_item = parent
        self._name = name
        self.setIcon(nm.settings().icon('state_off.png'))
        self.descr_type = self.descr_name = self.descr = ''
        self.descr_images = []
        self._capcabilities = dict()
        self._has_remote_launched_nodes = has_remote_launched_nodes
        self._remote_launched_nodes_updated = False
        '''
        :ivar: dict(config : dict(namespace: dict(group:dict('type' : str, 'images' : [str], 'description' : str, 'nodes' : [str]))))
        '''
        self._re_cap_nodes = dict()
        self._is_group = is_group
        self._state = NodeItem.STATE_OFF
        self.diagnostic_level = 0
        self.is_system_group = name == 'SYSTEM'
        self._clearup_mark_delete = False

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
        Set the new name of this group and updates the displayed name of the item.

        :param str new_name: The new name of the group. Used also to identify the group.
        '''
        self._name = new_name
        if self._is_group:
            self.setText('{' + self._name + '}')
        else:
            self.setText(self._name + '/')

    @property
    def state(self):
        '''
        The state of this group.

        :rtype: int
        '''
        return self._state

    @property
    def is_group(self):
        return self._is_group

    @property
    def cfgs(self):
        return self.get_configs()

    def get_namespace(self):
        name = self._name
        if type(self) == HostItem:
            name = rospy.names.SEP
        elif type(self) == GroupItem and self._is_group:
            name = namespace(self._name)
        result = name
        if self.parent_item is not None:
            result = normns(self.parent_item.get_namespace() + rospy.names.SEP) + normns(result + rospy.names.SEP)
        return normns(result)

    def count_nodes(self):
        '''
        :retrun: Returns count of nodes inside this group.
        :rtype: int
        '''
        result = 0
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                result += item.count_nodes()
            elif isinstance(item, NodeItem):
                result += 1
        return result

    def is_in_cap_group(self, nodename, config, ns, groupname):
        '''
        Returns `True` if the group contains the node.

        :param str nodename: the name of the node to test
        :param str config: the configuration name
        :param str ns: namespace
        :param str groupname: the group name
        :return: `True`, if the nodename is in the group
        :rtype: bool
        '''
        try:
            if self._re_cap_nodes[(config, ns, groupname)].match(nodename):
                return True
        except Exception:
            pass
        return False

    def _create_cap_nodes_pattern(self, config, cap):
        for ns, groups in cap.items():
            for groupname, descr in groups.items():
                try:
                    nodes = descr['nodes']
                    def_list = ['\A' + n.strip().replace('*', '.*') + '\Z' for n in nodes]
                    if def_list:
                        self._re_cap_nodes[(config, ns, groupname)] = re.compile('|'.join(def_list), re.I)
                    else:
                        self._re_cap_nodes[(config, ns, groupname)] = re.compile('\b', re.I)
                except Exception:
                    rospy.logwarn("create_cap_nodes_pattern: %s" % traceback.format_exc(1))

    def add_capabilities(self, config, capabilities, masteruri):
        '''
        Add new capabilities. Based on this capabilities the node are grouped. The
        view will be updated.

        :param str config: The name of the configuration containing this new capabilities.
        :param str masteruri: The masteruri is used only used, if new nodes are created.
        :param capabilities: The capabilities, which defines groups and containing nodes.
        :type capabilities: dict(namespace: dict(group: dict('type': str, 'images': list(str), 'description': str, 'nodes': list(str))))
        '''
        self._capcabilities[config] = capabilities
        self._create_cap_nodes_pattern(config, capabilities)
        # update the view
        for ns, groups in capabilities.items():
            for group, descr in groups.items():
                group_changed = False
                # create nodes for each group
                nodes = descr['nodes']
                if nodes:
                    groupItem = self.get_group_item(roslib.names.ns_join(ns, group), nocreate=False)
                    groupItem.descr_name = group
                    if descr['type']:
                        groupItem.descr_type = descr['type']
                    if descr['description']:
                        groupItem.descr = descr['description']
                    if descr['images']:
                        groupItem.descr_images = list(descr['images'])
                    # move the nodes from host to the group
                    group_changed = self.move_nodes2group(groupItem, config, ns, group, self)
                    # create new or update existing items in the group
                    for node_name in nodes:
                        # do not add nodes with * in the name
                        if not re.search(r"\*", node_name):
                            items = groupItem.get_node_items_by_name(node_name)
                            if items:
                                for item in items:
                                    item.add_config(config)
                                    group_changed = True
                            else:
                                items = self.get_node_items_by_name(node_name)
                                if items:
                                    # copy the state of the existing node
                                    groupItem.add_node(items[0].node_info, config)
                                elif config:
                                    groupItem.add_node(NodeInfo(node_name, masteruri), config)
                                group_changed = True
                    if group_changed:
                        groupItem.update_displayed_config()
                        groupItem.updateIcon()

    def move_nodes2group(self, group_item, config, ns, groupname, host_item):
        '''
        Returns `True` if the group was changed by adding a new node.

        :param GroupItem group_item: item to parse the children for nodes.
        :param str config: the configuration name
        :param str ns: namespace
        :param str groupname: the group name
        :param HostItem host_item: the host item contain the capability groups
        :return: `True`, if the group was changed by adding a new node.
        :rtype: bool
        '''
        self_changed = False
        group_changed = False
        for i in reversed(range(self.rowCount())):
            item = self.child(i)
            if isinstance(item, NodeItem):
                if host_item.is_in_cap_group(item.name, config, ns, groupname):
                    row = self.takeRow(i)
                    group_item._add_row_sorted(row)
                    group_changed = True
            elif isinstance(item, GroupItem) and not item.is_group:
                group_changed = item.move_nodes2group(group_item, config, ns, groupname, host_item)
        if self_changed:
            self.update_displayed_config()
            self.updateIcon()
        return group_changed

    def rem_capablities(self, config):
        '''
        Removes internal entry of the capability, so the new nodes are not grouped.
        To update view :meth:`NodeTreeModel.remove_config` and :meth:`GroupItem.clearup`
        must be called.

        :param str config: The name of the configuration containing this new capabilities.
        '''
        try:
            del self._capcabilities[config]
        except Exception:
            pass
        else:
            # todo update view?
            pass

    def get_capability_groups(self, node_name):
        '''
        Returns the names of groups, which contains the given node.

        :param str node_name: The name of the node
        :return: The name of the configuration containing this new capabilities.
        :rtype: dict(config : list(str))
        '''
        result = dict()  # dict(config : [group names])
        try:
            for cfg, cap in self._capcabilities.items():
                for ns, groups in cap.items():
                    for group, _ in groups.items():  # _:=decription
                        if self.is_in_cap_group(node_name, cfg, ns, group):
                            if cfg not in result:
                                result[cfg] = []
                            result[cfg].append(roslib.names.ns_join(ns, group))
        except Exception:
            pass
        return result

    def exists_capability_group(self, ns, group_name):
        '''
        Returns True if the group exists in capability list.

        :param str ns: Namespace of the group
        :param str group_name: The name of the group
        :return: True if the group exists in capability list.
        :rtype: bool
        '''
        try:
            if type(self) == HostItem:
                # replace last namespace separator if it is not the only one
                if len(ns) > 1:
                    ns = ns.rstrip(rospy.names.SEP)
                for _cfg, cap in self._capcabilities.items():
                    for gns, groups in cap.items():
                        for group, _decription in groups.items():
                            if ns == gns and group == group_name:
                                return True
            elif self.parent_item is not None:
                return self.parent_item.exists_capability_group(ns, group_name)
        except Exception:
            pass
        return False

    def clear_multiple_screens(self):
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                item.clear_multiple_screens()
            elif isinstance(item, NodeItem):
                item.has_multiple_screens = False

    def get_node_items_by_name(self, node_name, recursive=True):
        '''
        Since the same node can be included by different groups, this method searches
        for all nodes with given name and returns these items.

        :param str node_name: The name of the node
        :param bool recursive: Searches in (sub) groups
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                if recursive:
                    result[len(result):] = item.get_node_items_by_name(node_name)
            elif isinstance(item, NodeItem) and item == node_name:
                return [item]
        return result

    def get_node_items_by_cfg(self, cfg):
        '''
        Returns all nodes with config in this group and subgroups.

        :param str cfg: returns the nodes for given config
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                    result[len(result):] = item.get_node_items_by_cfg(cfg)
            elif isinstance(item, NodeItem):
                if cfg in item.cfgs:
                    result.append(item)
        return result


    def get_node_items(self, recursive=True):
        '''
        Returns all nodes in this group and subgroups.

        :param bool recursive: returns the nodes of the subgroups
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                if recursive:
                    result[len(result):] = item.get_node_items()
            elif isinstance(item, NodeItem):
                result.append(item)
        return result

    def get_group_item(self, group_name, is_group=True, nocreate=False):
        '''
        Returns a GroupItem with given name. If no group with this name exists, a
        new one will be created. The given name will be split by slashes if exists
        and subgroups are created.

        :param str group_name: the name of the group
        :param bool is_group: True if it is a capability group. False if a namespace group. (Default: True)
        :param bool nocreate: avoid creation of new group if not exists. (Default: False)
        :return: The group with given name of None if `nocreate` is True and group not exists.
        :rtype: :class:`GroupItem` or None
        '''
        lns, rns = group_name, ''
        if nm.settings().group_nodes_by_namespace:
            lns, rns = lnamespace(group_name)
            if lns == rospy.names.SEP and type(self) == HostItem:
                lns, rns = lnamespace(rns)
        if lns == rospy.names.SEP:
            return self
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                if item == lns and not item._clearup_mark_delete:
                    if rns:
                        return item.get_group_item(rns, is_group, nocreate)
                    return item
                elif item > lns and not nocreate:
                    items = []
                    newItem = GroupItem(lns, self, is_group=(is_group and not rns))
                    items.append(newItem)
                    cfgitem = CellItem(group_name, newItem)
                    items.append(cfgitem)
                    self.insertRow(i, items)
                    if rns:
                        return newItem.get_group_item(rns, is_group, nocreate)
                    return newItem
        if nocreate:
            return None
        items = []
        newItem = GroupItem(lns, self, is_group=(is_group and not rns))
        items.append(newItem)
        cfgitem = CellItem(group_name, newItem)
        items.append(cfgitem)
        self.appendRow(items)
        if rns:
            return newItem.get_group_item(rns, is_group, nocreate)
        return newItem

    def add_node(self, node, cfg=None):
        '''
        Adds a new node with given name.

        :param node: the NodeInfo of the node to create
        :type node: :class:`NodeInfo`
        :param str cfg: The configuration, which describes the node
        '''
        groups = self.get_capability_groups(node.name)
        if groups:
            for _, group_list in groups.items():
                for group_name in group_list:
                    # insert in the group
                    groupItem = self.get_group_item(group_name, is_group=True)
                    groupItem.add_node(node, cfg)
        else:
            group_item = self
            if type(group_item) == HostItem:
                # insert in the group
                group_item = self.get_group_item(namespace(node.name), is_group=False)
            # insert in order
            new_item_row = NodeItem.newNodeRow(node.name, node.masteruri)
            group_item._add_row_sorted(new_item_row)
            new_item_row[0].set_node_info(node)
            if cfg or cfg == '':
                new_item_row[0].add_config(cfg)
            group_item.updateIcon()

    def _add_row_sorted(self, row):
        for i in range(self.rowCount()):
            item = self.child(i)
            if item > row[0].name:
                self.insertRow(i, row)
                row[0].parent_item = self
                return
        self.appendRow(row)
        row[0].parent_item = self

    def clearup(self, fixed_node_names=None):
        '''
        Removes not running and not configured nodes.

        :param list(str) fixed_node_names: If the list is not None, the node not in the list are set to not running!
        '''
        self._clearup(fixed_node_names)
        self._mark_groups_to_delete()
        self._remove_marked_groups()

    def _clearup(self, fixed_node_names=None):
        '''
        Removes not running and not configured nodes.

        :param list(str) fixed_node_names: If the list is not None, the node not in the list are set to not running!
        '''
        removed = False
        # move running nodes without configuration to the upper layer, remove not running and duplicate nodes
        for i in reversed(range(self.rowCount())):
            item = self.child(i)
            if isinstance(item, NodeItem):
                # set the running state of the node to None
                if fixed_node_names is not None:
                    if item.name not in fixed_node_names:
                        item.set_node_info(NodeInfo(item.name, item.node_info.masteruri))
                if not (item.has_configs() or item.is_running() or item.published or item.subscribed or item.services):
                    removed = True
                    self._remove_row(i)
            else:  # if type(item) == GroupItem:
                removed = item._clearup(fixed_node_names) or removed
        if self.rowCount() == 0 and self.parent_item is not None:
            self.parent_item._remove_group(self.name)
        elif removed:
            self.updateIcon()
        return removed

    def _mark_groups_to_delete(self):
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, NodeItem):
                # remove group if only one node is inside
                if not isinstance(self, HostItem):
                    if self.parent_item is not None:
                        if self.is_group:
                            # check to remove capability group
                            if not self.exists_capability_group(self.parent_item.get_namespace(), self.name):
                                self._clearup_mark_delete = True
                        elif self.rowCount() == 1:
                            self._clearup_mark_delete = True
            else:
                item._mark_groups_to_delete()
                if self.rowCount() == 1:
                    # remove if subgroup marked to remove and this has only this group
                    self._clearup_mark_delete = item._clearup_mark_delete

    def _remove_marked_groups(self):
        rows2add = []
        for i in reversed(range(self.rowCount())):
            item = self.child(i)
            if isinstance(item, GroupItem):
                if item._clearup_mark_delete:
                    rows = self._take_node_rows(item)
                    if rows:
                        rows2add = rows2add + rows
                        self._remove_row(i)
                else:
                    item._remove_marked_groups()
        for row in rows2add:
            self._add_row_sorted(row)
            self.updateIcon()

    def _take_node_rows(self, group):
        result = []
        for i in reversed(range(group.rowCount())):
            item = group.child(i)
            if isinstance(item, NodeItem):
                result.append(group.takeRow(i))
            else:
                result = result + item._take_node_rows(item)
        return result

    def _remove_group(self, name):
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == GroupItem and item == name and item.rowCount() == 0:
                self._remove_row(i)
                return  # we assume only one group with same name can exists

    def _remove_row(self, index):
        item = self.child(index)
        item.parent_item = None
        try:
            cellitem = self.child(index, 1)
            cellitem.parent_item = None
            cellitem.item = None
        except Exception as e:
            rospy.logdebug_throttle(10, utf8(e))
        self.removeRow(index)

    def reset_remote_launched_nodes(self):
        self._remote_launched_nodes_updated = False

    def remote_launched_nodes_updated(self):
        if self._has_remote_launched_nodes:
            return self._remote_launched_nodes_updated
        return True

    def update_running_state(self, nodes, create_nodes=True):
        '''
        Updates the running state of the nodes given in a dictionary.

        :param nodes: A dictionary with node names and their running state described by L{NodeInfo}.
        :type nodes: dict(str: :class:`fkie_master_discovery.NodeInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.NodeInfo>)
        :return: a list with :class:`fkie_master_discovery.NodeInfo` items, which are changed their PID or URI.
        '''
        updated_nodes = []
        if isinstance(nodes, dict):
            for (name, node) in nodes.items():
                # get the node items
                items = self.get_node_items_by_name(name)
                if items:
                    for item in items:
                        # update the node item
                        run_changed = item.set_node_info(node)
                        #updated_nodes.append(node)
                        if run_changed:
                            updated_nodes.append(node)
                elif create_nodes:
                    # create the new node
                    self.add_node(node)
                    updated_nodes.append(node)
                if self._has_remote_launched_nodes:
                    self._remote_launched_nodes_updated = True
            self.clearup(list(nodes.keys()))
        elif isinstance(nodes, list):
            self.clearup(nodes)
        return updated_nodes

    def get_nodes_running(self):
        '''
        Returns the names of all running nodes. A running node is defined by his
        PID.

        :see: :class:`master_dicovery_fkie.NodeInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.NodeInfo>
        :return: A list with node names
        :rtype: list(str)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                result[len(result):] = item.get_nodes_running()
            elif isinstance(item, NodeItem) and item.node_info.pid is not None:
                result.append(item.name)
        return result

    def set_duplicate_nodes(self, running_nodes, is_sync_running=False):
        '''
        While a synchronization same node on different hosts have the same name, the
        nodes with the same on other host are marked.

        :param running_nodes: The dictionary with names of running nodes and their masteruri
        :type running_nodes: dict(str: str)
        :param bool is_sync_running: If the master_sync is running, the nodes are marked
          as ghost nodes. So they are handled as running nodes, but has not run
          informations. This nodes are running on remote host, but are not
          syncronized because of filter or errors.
        '''
        ignore = ['/master_sync', '/master_discovery', '/node_manager', '/node_manager_daemon']
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, GroupItem):
                item.set_duplicate_nodes(running_nodes, is_sync_running)
            elif isinstance(item, NodeItem):
                if is_sync_running:
                    item.is_ghost = (item.node_info.uri is None and (item.name in running_nodes and running_nodes[item.name] == item.node_info.masteruri))
                    item.has_running = (item.node_info.uri is None and item.name not in ignore and (item.name in running_nodes and running_nodes[item.name] != item.node_info.masteruri))
                else:
                    if item.is_ghost:
                        item.is_ghost = False
                    item.has_running = (item.node_info.uri is None and item.name not in ignore and (item.name in running_nodes))

    def updateIcon(self):
        if isinstance(self, HostItem):
            # skip the icon update on a host item
            return
        has_running = False
        has_off = False
        has_duplicate = False
        has_ghosts = False
        self.diagnostic_level = 0
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, (GroupItem, NodeItem)):
                if item.state == NodeItem.STATE_WARNING:
                    self.setIcon(nm.settings().icon('crystal_clear_warning.png'))
                    self._state = NodeItem.STATE_WARNING
                    if self.parent_item is not None:
                        self.parent_item.updateIcon()
                    return
                elif item.state == NodeItem.STATE_OFF:
                    has_off = True
                elif item.state == NodeItem.STATE_RUN:
                    has_running = True
                elif item.state == NodeItem.STATE_GHOST:
                    has_ghosts = True
                elif item.state == NodeItem.STATE_DUPLICATE:
                    has_duplicate = True
                elif item.state == NodeItem.STATE_PARTS:
                    has_running = True
                    has_off = True
                if item.state == NodeItem.STATE_RUN or isinstance(item, GroupItem):
                    if item.diagnostic_level > self.diagnostic_level:
                        self.diagnostic_level = item.diagnostic_level
        if self.diagnostic_level > 0:
            self.setIcon(NodeItem._diagnostic_level2icon(self.diagnostic_level))
        else:
            if has_duplicate:
                self._state = NodeItem.STATE_DUPLICATE
                self.setIcon(nm.settings().icon('imacadam_stop.png'))
            elif has_ghosts:
                self._state = NodeItem.STATE_GHOST
                self.setIcon(nm.settings().icon('state_ghost.png'))
            elif has_running and has_off:
                self._state = NodeItem.STATE_PARTS
                self.setIcon(nm.settings().icon('state_part.png'))
            elif not has_running:
                self._state = NodeItem.STATE_OFF
                self.setIcon(nm.settings().icon('state_off.png'))
            elif not has_off and has_running:
                self._state = NodeItem.STATE_RUN
                self.setIcon(nm.settings().icon('state_run.png'))
        if self.parent_item is not None:
            self.parent_item.updateIcon()

    def _create_html_list(self, title, items):
        result = ''
        if items:
            result += '<b><u>%s</u></b>' % title
            if len(items) > 1:
                result += ' <span style="color:gray;">[%d]</span>' % len(items)
            result += '<ul><span></span><br>'
            for i in items:
                result += '<a href="node://%s">%s</a><br>' % (i, i)
            result += '</ul>'
        return result

    def update_tooltip(self):
        '''
        Creates a tooltip description based on text set by :meth:`update_description`
        and all childs of this host with valid sensor description. The result is
        returned as a HTML part.

        :return: the tooltip description coded as a HTML part
        :rtype: str
        '''
        tooltip = self.generate_description(False)
        self.setToolTip(tooltip if tooltip else self.name)
        return tooltip

    def generate_description(self, extended=True):
        tooltip = ''
        if self.descr_type or self.descr_name or self.descr:
            tooltip += '<h4>%s</h4><dl>' % self.descr_name
            if self.descr_type:
                tooltip += '<dt>Type: %s</dt></dl>' % self.descr_type
            if extended:
                try:
                    from docutils import examples
                    if self.descr:
                        tooltip += '<b><u>Detailed description:</u></b>'
                        tooltip += examples.html_body(utf8(self.descr))
                except Exception:
                    rospy.logwarn("Error while generate description for a tooltip: %s", traceback.format_exc(1))
                    tooltip += '<br>'
            # get nodes
            nodes = []
            for j in range(self.rowCount()):
                nodes.append(self.child(j).name)
            if nodes:
                tooltip += self._create_html_list('Nodes:', nodes)
        return '<div>%s</div>' % tooltip

    def update_description(self, descr_type, descr_name, descr):
        '''
        Sets the description of the robot. To update the tooltip of the host item use :meth:`update_tooltip`.

        :param str descr_type: the type of the robot
        :param str descr_name: the name of the robot
        :param str descr: the description of the robot as a reStructuredText<http://docutils.sourceforge.net/rst.html>
        '''
        self.descr_type = descr_type
        self.descr_name = descr_name
        self.descr = descr

    def update_displayed_config(self):
        '''
        Updates the configuration representation in other column.
        '''
        if self.parent_item is not None:
            # get nodes
            cfgs = []
            for j in range(self.rowCount()):
                if self.child(j).cfgs:
                    cfgs[len(cfgs):] = self.child(j).cfgs
            if cfgs:
                cfgs = list(set(cfgs))
            cfg_col = self.parent_item.child(self.row(), NodeItem.COL_CFG)
            if cfg_col is not None and isinstance(cfg_col, QStandardItem):
                cfg_col.setText('[%d]' % len(cfgs) if len(cfgs) > 1 else "")
                # set tooltip
                # removed for clarity !!!
#        tooltip = ''
#        if len(cfgs) > 0:
#          tooltip = ''
#          if len(cfgs) > 0:
#            tooltip = ''.join([tooltip, '<h4>', 'Configurations:', '</h4><dl>'])
#            for c in cfgs:
#              if NodeItem.is_default_cfg(c):
#                tooltip = ''.join([tooltip, '<dt>[default]', c[0], '</dt>'])
#              else:
#                tooltip = ''.join([tooltip, '<dt>', c, '</dt>'])
#            tooltip = ''.join([tooltip, '</dl>'])
#        cfg_col.setToolTip(''.join(['<div>', tooltip, '</div>']))
                # set icons
                has_launches = NodeItem.has_launch_cfgs(cfgs)
                has_defaults = NodeItem.has_default_cfgs(cfgs)
                if has_launches and has_defaults:
                    cfg_col.setIcon(nm.settings().icon('crystal_clear_launch_file_def_cfg.png'))
                elif has_launches:
                    cfg_col.setIcon(nm.settings().icon('crystal_clear_launch_file.png'))
                elif has_defaults:
                    cfg_col.setIcon(nm.settings().icon('default_cfg.png'))
                else:
                    cfg_col.setIcon(QIcon())

    def get_configs(self):
        '''
        Returns the set for launch configurations.

        :rtype: set(str)
        '''
        cfgs = []
        for j in range(self.rowCount()):
            if isinstance(self.child(j), GroupItem):
                cfgs[len(cfgs):] = self.child(j).get_configs()
            elif self.child(j).cfgs:
                cfgs[len(cfgs):] = self.child(j).cfgs
        return set(cfgs)

    def get_count_mscreens(self):
        '''
        Returns count for nodes with multiple screens
        '''
        result = 0
        for j in range(self.rowCount()):
            if isinstance(self.child(j), GroupItem):
                result += self.child(j).get_count_mscreens()
            elif self.child(j).has_multiple_screens:
                result += 1
        return result

    def type(self):
        return GroupItem.ITEM_TYPE

    def __eq__(self, item):
        '''
        Compares the name of the group.
        '''
        if isstring(item):
            return self.name.lower() == item.lower()
        elif item is not None and type(item) == GroupItem:
            return self.name.lower() == item.name.lower()
        return False

    def __ne__(self, item):
        return not (self == item)

    def __gt__(self, item):
        '''
        Compares the name of the group.
        '''
        if isstring(item):
            # put the group with SYSTEM nodes at the end
            if self.is_system_group:
                if self.name.lower() != item.lower():
                    return True
            elif item.lower() == 'system':
                return False
            return self.name.lower() > item.lower()
        elif item is not None and type(item) == GroupItem:
            # put the group with SYSTEM nodes at the end
            if item.is_system_group:
                if self.name.lower() != item.lower():
                    return True
            elif self.is_syste_group:
                return False
            return self.name.lower() > item.name.lower()
        return False


# ###############################################################################
# #############                   HostItem                         ##############
# ###############################################################################

class HostItem(GroupItem):
    '''
    The HostItem stores the information about a host.
    '''
    ITEM_TYPE = Qt.UserRole + 26

    def __init__(self, masteruri, address, local, master_entry, parent=None):
        '''
        Initialize the HostItem object with given values.

        :param str masteruri: URI of the ROS master assigned to the host
        :param str address: the address of the host
        :param bool local: is this host the localhost where the node_manager is running.
        '''
        self._has_remote_launched_nodes = False
        self._masteruri = masteruri
        self._host = address
        self._mastername = address
        self._master_entry = master_entry
        self._local = None
        self._diagnostics = []
        name = self.create_host_description(master_entry)
        GroupItem.__init__(self, name, parent, has_remote_launched_nodes=self._has_remote_launched_nodes)
        self.descr_type = self.descr_name = self.descr = ''
        self.sysmon_state = False
        self.local = local

    def __hash__(self):
        str = self._masteruri + self._host
        hash_str = hashlib.md5(str.encode()).hexdigest()
        return int(hash_str, base=16)

    @property
    def host(self):
        return self._host

    @property
    def hostname(self):
        # return nm.nameres().hostname(self._host)
        return self._host

    @property
    def addresses(self):
        return nm.nameres().resolve_cached(self._host)

    @property
    def masteruri(self):
        return self._masteruri

    @property
    def mastername(self):
        result = self._master_entry.get_mastername()
        if result is None or not result:
            result = self.hostname
        return result

    @property
    def local(self):
        return self._local

    @local.setter
    def local(self, islocal):
        if self._local != islocal:
            self._local = islocal
            name = self.create_host_description(self._master_entry)
            image_file = nm.settings().robot_image_file(name)
            if QFile.exists(image_file):
                self.setIcon(QIcon(image_file))
            else:
                if self._local:
                    self.setIcon(nm.settings().icon('crystal_clear_miscellaneous.png'))
                else:
                    self.setIcon(nm.settings().icon('remote.png'))

    @property
    def diagnostics(self):
        return list(self._diagnostics)

    def update_system_diagnostics(self, diagnostics):
        del self._diagnostics[:]
        for diagnostic in diagnostics.status:
            if diagnostic.hardware_id == self.hostname:
                self._diagnostics.append(diagnostic)
        self.update_tooltip()
        self.name = self.create_host_description(self._master_entry)

    def create_host_description(self, master_entry):
        '''
        Returns the name generated from masteruri and address

        :param masteruri: URI of the ROS master assigned to the host
        :param str address: the address of the host
        '''
        name = master_entry.get_mastername()
        if not name:
            name = master_entry.get_address()
        hostname = master_entry.get_address()
        if not nm.settings().show_domain_suffix:
            name = subdomain(name)
        result = '%s@%s' % (name, hostname)
        maddr = get_hostname(master_entry.masteruri)
        mname = nm.nameres().hostname(maddr)
        if mname is None:
            mname = utf8(maddr)
        if mname != hostname:
            result += '[%s]' % master_entry.masteruri
            self._has_remote_launched_nodes = True
        return result

    def update_tooltip(self):
        '''
        Creates a tooltip description based on text set by :meth:`update_description`
        and all childs of this host with valid sensor description. The result is
        returned as a HTML part.

        :return: the tooltip description coded as a HTML part
        :rtype: str
        '''
        tooltip = self.generate_description(False)
        self.setToolTip(tooltip if tooltip else self.name)
        return tooltip

    def generate_description(self, extended=True):
        from docutils import examples
        tooltip = ''
        if self.descr_type or self.descr_name or self.descr:
            tooltip += '<h4>%s</h4><dl>' % self.descr_name
            if self.descr_type:
                tooltip += '<dt>Type: %s</dt></dl>' % self.descr_type
            if extended:
                try:
                    if self.descr:
                        tooltip += '<b><u>Detailed description:</u></b>'
                        tooltip += examples.html_body(self.descr, input_encoding='utf8')
                except Exception:
                    rospy.logwarn("Error while generate description for a tooltip: %s", traceback.format_exc(1))
                    tooltip += '<br>'
        tooltip += '<h3>%s</h3>' % self.mastername
        tooltip += '<font size="+1"><i>%s</i></font><br>' % self.masteruri
        tooltip += '<font size="+1">Host: <b>%s%s</b></font><br>' % (self.hostname, ' %s' % self.addresses if self.addresses else '')
        if extended:
            tooltip += '<a href="open-sync-dialog://%s">open sync dialog</a>' % (utf8(self.masteruri).replace('http://', ''))
            tooltip += '<p>'
            tooltip += '<a href="show-all-screens://%s">show all screens</a>' % (utf8(self.masteruri).replace('http://', ''))
            tooltip += '<p>'
            tooltip += '<a href="rosclean://%s" title="calls `rosclean purge` at `%s`">rosclean purge</a>' % (self.masteruri.replace('http://', ''), self.hostname)
            tooltip += '<p>'
            tooltip += '<a href="poweroff://%s" title="calls `sudo poweroff` at `%s` via SSH">poweroff `%s`</a>' % (self.hostname, self.hostname, self.hostname)
            tooltip += '<p>'
            tooltip += '<a href="remove-all-launch-server://%s">kill all launch server</a>' % utf8(self.masteruri).replace('http://', '')
            tooltip += '<p>'
            if self.local:
                icon_path_settings = nm.settings().icon_path('crystal_clear_settings_24.png')
                sysmon_setup_str = '<a href="nmd-cfg://%s" title="Configure Daemon"><img src="%s" alt="configure"></a>' % (utf8(self.masteruri).replace('http://', ''), icon_path_settings)
                sysmon_state_str = 'disable' if self.sysmon_state else 'enable'
                sysmon_switch_str = '<a href="sysmon-switch://%s">%s</a>' % (utf8(self.masteruri).replace('http://', ''), sysmon_state_str)
                tooltip += '<h3>System Monitoring: (%s) %s</h3>' % (sysmon_switch_str, sysmon_setup_str)
                if self._diagnostics:
                    for diag in self._diagnostics:
                        try:
                            free = None
                            free_percent = None
                            stamp = None
                            others = []
                            for val in diag.values:
                                if val.key == 'Free [%]':
                                    free_percent = float(val.value)
                                elif val.key == 'Free':
                                    free = sizeof_fmt(float(val.value))
                                elif val.key == 'Timestamp':
                                    stamp = val.value
                                else:
                                    others.append((val.key, val.value))
                            tooltip += '\n<b>%s:</b> <font color=grey>%s</font>' % (diag.name, stamp)
                            if diag.level > 0:
                                tooltip += '\n<dt><font color="red">%s</font></dt>' % (diag.message.replace('>', '&gt;').replace('<', '&lt;'))
                            else:
                                tooltip += '\n<dt><font color="grey">%s</font></dt>' % (diag.message.replace('>', '&gt;').replace('<', '&lt;'))
                            if free is not None:
                                tooltip += '\n<dt><em>%s:</em> %s (%s%%)</dt>' % ('Free', free, free_percent)
                            cpu_processes = 3
                            for key, value in others:
                                key_fmt = key
                                val_fmt = value
                                if '[1s]' in key:
                                    val_fmt = '%s/s' % sizeof_fmt(float(value))
                                    key_fmt = key_fmt.replace(' [1s]', '')
                                elif '[%]' in key:
                                    val_fmt = '%s%%' % value
                                    key_fmt = key_fmt.replace(' [%]', '')
                                elif '[degree]' in key:
                                    val_fmt = '%s&deg;C' % value
                                    key_fmt = key_fmt.replace(' [degree]', '')
                                if key == 'Process load':
                                    kill_ref = ''
                                    pid = self._pid_from_str(val_fmt)
                                    if pid:
                                        kill_ref = ' <a href="kill-pid://pid%s">kill</a>' % pid
                                    tooltip += '\n<dt><font color="red">%s</font>%s</dt>' % (val_fmt, kill_ref)
                                    cpu_processes -= 1
                                else:
                                    tooltip += '\n<dt><em>%s:</em> %s</dt>' % (key_fmt, val_fmt)
                            if cpu_processes > 0 and diag.name == 'CPU Load':
                                for _idx in range(cpu_processes):
                                    tooltip += '\n<dt><font color="grey">%s</font></dt>' % ('--')
                        except Exception as err:
                            tooltip += '\n<dt><font color="red">%s</font></dt>' % (utf8(err))
                        tooltip += '<br>'

        # get sensors
        capabilities = []
        for j in range(self.rowCount()):
            item = self.child(j)
            if isinstance(item, GroupItem):
                capabilities.append(item.name)
        if capabilities:
            tooltip += '<br>'
            tooltip += '<b><u>Capabilities:</u></b>'
            try:
                tooltip += examples.html_body('- %s' % ('\n- '.join(capabilities)), input_encoding='utf8')
            except Exception:
                rospy.logwarn("Error while generate description for a tooltip: %s", traceback.format_exc(1))
        return '<div>%s</div>' % tooltip if tooltip else ''

    def _pid_from_str(self, string):
        re_if = re.compile(r".*\[(?P<pid>.*?)\]")
        for pid in re_if.findall(string):
            return pid
        return ''

    def type(self):
        return HostItem.ITEM_TYPE

    def __eq__(self, item):
        '''
        Compares the address of the masteruri.
        '''
        if isstring(item):
            rospy.logwarn("compare HostItem with unicode deprecated")
            return False
        elif isinstance(item, tuple):
            return nmdurl.equal_uri(self.masteruri, item[0]) and self.host == item[1]
        elif isinstance(item, MasterEntry):
            return self._master_entry == item
        elif isinstance(item, HostItem):
            return self._master_entry == item._master_entry
        return False

    def __gt__(self, item):
        '''
        Compares the address of the masteruri.
        '''
        if isstring(item):
            rospy.logwarn("compare HostItem with unicode deprecated")
            return False
        elif isinstance(item, tuple):
            return self.masteruri > item[0]
        elif isinstance(item, HostItem):
            return self.masteruri > item.masteruri
        return False


# ###############################################################################
# #############                   NodeItem                         ##############
# ###############################################################################

class NodeItem(QStandardItem):
    '''
    The NodeItem stores the information about the node using the ExtendedNodeInfo
    class and represents it in a :class:`QTreeView`<https://srinikom.github.io/pyside-docs/PySide/QtGui/QTreeView.html> using the
    :class:`QStandardItemModel` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItemModel.html>
    '''

    ITEM_TYPE = QStandardItem.UserType + 35
    NAME_ROLE = Qt.UserRole + 1
    COL_CFG = 1
#  COL_URI = 2

    STATE_OFF = 0
    STATE_RUN = 1
    STATE_WARNING = 2
    STATE_GHOST = 3
    STATE_DUPLICATE = 4
    STATE_PARTS = 5

    def __init__(self, node_info):
        '''
        Initialize the NodeItem instance.

        :param node_info: the node information
        :type node_info: :class:`fkie_master_discovery.NodeInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.NodeInfo>
        '''
        QStandardItem.__init__(self, node_info.name)
        self._parent_item = None
        self._node_info = node_info.copy()
#    self.ICONS = {'empty' : QIcon(),
#                  'run'    : nm.settings().icon('state_run.png'),
#                  'off'     :nm.settings().icon('state_off.png'),
#                  'warning' : nm.settings().icon('crystal_clear_warning.png'),
#                  'stop'    : QIcon('icons/imacadam_stop.png'),
#                  'cfg+def' : nm.settings().icon('crystal_clear_launch_file_def_cfg.png'),
#                  'cfg'     : nm.settings().icon('crystal_clear_launch_file.png'),
#                  'default_cfg' : nm.settings().icon('default_cfg.png')
#                  }
        self._cfgs = []
        self.launched_cfg = None  # is used to store the last configuration to launch the node
        self.next_start_cfg = None  # is used to set the configuration for next start of the node
        self._std_config = None  # it's config with empty name. for default proposals
        self._is_ghost = False
        self._has_running = False
        self.setIcon(nm.settings().icon('state_off.png'))
        self._state = NodeItem.STATE_OFF
        self.diagnostic_array = []
        self.nodelet_mngr = ''
        self.nodelets = []
        self.has_screen = True
        self.has_multiple_screens = False
        self._with_namespace = rospy.names.SEP in node_info.name
        self.kill_on_stop = False
        self._kill_parameter_handler = ParameterHandler()
        self._kill_parameter_handler.parameter_values_signal.connect(self._on_kill_param_values)

    @property
    def state(self):
        return self._state

    @property
    def name(self):
        return self._node_info.name

    @name.setter
    def name(self, new_name):
        self.setText(new_name)

    @property
    def masteruri(self):
        return self._node_info.masteruri

    @property
    def published(self):
        return self._node_info.publishedTopics

    @property
    def subscribed(self):
        return self._node_info.subscribedTopics

    @property
    def services(self):
        return self._node_info.services

    @property
    def parent_item(self):
        return self._parent_item

    @parent_item.setter
    def parent_item(self, parent_item):
        self._parent_item = parent_item
        if parent_item is None:
            self.setText(self._node_info.name)
            self._with_namespace = rospy.names.SEP in self._node_info.name
        else:
            new_name = self._node_info.name.replace(parent_item.get_namespace(), '', 1)
            self.setText(new_name)
            self._with_namespace = rospy.names.SEP in new_name

    @property
    def node_info(self):
        '''
        Returns the NodeInfo instance of this node.

        :rtype: :class:`fkie_master_discovery.NodeInfo` <http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.NodeInfo>
        '''
        return self._node_info

    def set_node_info(self, node_info):
        '''
        Sets the NodeInfo and updates the view, if needed.
        '''
        abbos_changed = False
        run_changed = False
        # print "!!!", self.name
        # print "  subs: ", self._node_info.subscribedTopics, node_info.subscribedTopics
        # print "  pubs: ", self._node_info.publishedTopics, node_info.publishedTopics
        # print "  srvs: ", self._node_info.services, node_info.services
        if self._node_info.publishedTopics != node_info.publishedTopics:
            abbos_changed = True
            self._node_info._publishedTopics = list(node_info.publishedTopics)
        if self._node_info.subscribedTopics != node_info.subscribedTopics:
            abbos_changed = True
            self._node_info._subscribedTopics = list(node_info.subscribedTopics)
        if self._node_info.services != node_info.services:
            abbos_changed = True
            self._node_info._services = list(node_info.services)
        if self._node_info.pid != node_info.pid:
            self._node_info.pid = node_info.pid
            run_changed = True
        if self._node_info.uri != node_info.uri:
            self._node_info.uri = node_info.uri
            run_changed = True
        # delete diagnostics messages on stop or start nodes
        if run_changed:
            del self.diagnostic_array[:]
        # update the tooltip and icon
        if run_changed and (self.is_running() or self.has_configs) or abbos_changed:
            self.has_screen = True
            self.update_dispayed_name()
#      self.update_displayed_url()
            if self.parent_item is not None:
                self.parent_item.updateIcon()
        if run_changed and self.is_running():
            # 'kill_on_stop' is deprecated
            self._kill_parameter_handler.requestParameterValues(self.masteruri, [roslib.names.ns_join(self.name, 'kill_on_stop'), roslib.names.ns_join(self.name, 'nm/kill_on_stop')])
            return True
        return False

    @property
    def uri(self):
        if self._node_info.uri is not None:
            if self._node_info.uri == 'None':
                self._node_info.uri = None
        return self._node_info.uri

    @property
    def pid(self):
        return self._node_info.pid

    @property
    def has_running(self):
        '''
        Returns `True`, if there are exists other nodes with the same name. This
        variable must be set manually!

        :rtype: bool
        '''
        return self._has_running

    @has_running.setter
    def has_running(self, state):
        '''
        Sets however other node with the same name are running or not (on other hosts)
        and updates the view of this item.
        '''
        if self._has_running != state:
            self._has_running = state
            if self.has_configs() or self.is_running():
                self.update_dispayed_name()
            if self.parent_item is not None and not isinstance(self.parent_item, HostItem):
                self.parent_item.updateIcon()

    @property
    def is_ghost(self):
        '''
        Returns `True`, if there are exists other runnig nodes with the same name. This
        variable must be set manually!

        :rtype: bool
        '''
        return self._is_ghost

    @is_ghost.setter
    def is_ghost(self, state):
        '''
        Sets however other node with the same name is running (on other hosts) and
        and the host showing this node the master_sync is running, but the node is
        not synchronized.
        '''
        if self._is_ghost != state:
            self._is_ghost = state
            if self.has_configs() or self.is_running():
                self.update_dispayed_name()
            if self.parent_item is not None and not isinstance(self.parent_item, HostItem):
                self.parent_item.updateIcon()

    @property
    def with_namespace(self):
        '''
        Returns `True` if the node name contains a '/' in his name

        :rtype: bool
        '''
        return self._with_namespace

    @property
    def host(self):
        pitem = self.parent_item
        while pitem is not None:
            if type(pitem) == HostItem:
                return pitem.host
            else:
                pitem = pitem.parent_item
        return None

    def append_diagnostic_status(self, diagnostic_status):
        if self.diagnostic_array:
            last_item = self.diagnostic_array[-1]
            if last_item.level == diagnostic_status.level:
                if last_item.message == diagnostic_status.message:
                    return
        dt_key = KeyValue()
        dt_key.key = 'recvtime'
        dt_key.value = datetime.now().strftime("%d.%m.%Y %H:%M:%S.%f")
        if diagnostic_status.values and diagnostic_status.values[-1].key == 'recvtime':
            diagnostic_status.values[-1].value = dt_key.value
        else:
            diagnostic_status.values.append(dt_key)
        self.diagnostic_array.append(diagnostic_status)
        self.update_dispayed_name()
        if self.parent_item is not None and not isinstance(self.parent_item, HostItem):
            self.parent_item.updateIcon()
        if len(self.diagnostic_array) > 15:
            del self.diagnostic_array[0]

    def data(self, role):
        if role == self.NAME_ROLE:
            return self.name
        else:
            return QStandardItem.data(self, role)

    @staticmethod
    def _diagnostic_level2icon(level):
        if level == 1:
            return nm.settings().icon('state_diag_warn.png')
        elif level == 2:
            return nm.settings().icon('state_diag_error.png')
        elif level == 3:
            return nm.settings().icon('state_diag_stale.png')
        else:
            return nm.settings().icon('state_diag_other.png')

    @property
    def diagnostic_level(self):
        if self.diagnostic_array:
            return self.diagnostic_array[-1].level
        return 0

    def _on_kill_param_values(self, masteruri, code, msg, params):
        if code == 1:
            # assumption: all parameter are 'kill_on_stop' parameter
            for _p, (code_n, _msg_n, val) in params.items():
                if code_n == 1:
                    self.kill_on_stop = val

    def update_dispayed_name(self):
        '''
        Updates the name representation of the Item
        '''
        tooltip = '<h4>%s</h4><dl>' % self.node_info.name
        tooltip += '<dt><b>URI:</b> %s</dt>' % self.node_info.uri
        tooltip += '<dt><b>PID:</b> %s</dt>' % self.node_info.pid
        if self.nodelet_mngr:
            tooltip += '<dt><b>Nodelet manager</b>: %s</dt>' % self.nodelet_mngr
        if self.nodelets:
            tooltip += '<dt><b>This is nodelet manager for %d nodes</b></dt>' % len(self.nodelets)
        tooltip += '<dt><b>ORG.MASTERURI:</b> %s</dt></dl>' % self.node_info.masteruri
        master_discovered = nm.nameres().has_master(self.node_info.masteruri)
#    local = False
#    if not self.node_info.uri is None and not self.node_info.masteruri is None:
#      local = (get_hostname(self.node_info.uri) == get_hostname(self.node_info.masteruri))
        if self.node_info.pid is not None:
            self._state = NodeItem.STATE_RUN
            if self.diagnostic_array and self.diagnostic_array[-1].level > 0:
                level = self.diagnostic_array[-1].level
                self.setIcon(self._diagnostic_level2icon(level))
                self.setToolTip(self.diagnostic_array[-1].message)
            else:
                self.setIcon(nm.settings().icon('state_run.png'))
                self.setToolTip('')
        elif self.node_info.uri is not None and not self.node_info.isLocal:
            self._state = NodeItem.STATE_RUN
            self.setIcon(nm.settings().icon('state_unknown.png'))
            tooltip += '<dl><dt>(Remote nodes will not be ping, so they are always marked running)</dt></dl>'
            tooltip += '</dl>'
            self.setToolTip('<div>%s</div>' % tooltip)
#    elif not self.node_info.isLocal and not master_discovered and not self.node_info.uri is None:
# #    elif not local and not master_discovered and not self.node_info.uri is None:
#      self._state = NodeItem.STATE_RUN
#      self.setIcon(nm.settings().icon('state_run.png'))
#      tooltip = ''.join([tooltip, '<dl><dt>(Remote nodes will not be ping, so they are always marked running)</dt></dl>'])
#      tooltip = ''.join([tooltip, '</dl>'])
#      self.setToolTip(''.join(['<div>', tooltip, '</div>']))
        elif self.node_info.pid is None and self.node_info.uri is None and (self.node_info.subscribedTopics or self.node_info.publishedTopics or self.node_info.services):
            self.setIcon(nm.settings().icon('crystal_clear_warning.png'))
            self._state = NodeItem.STATE_WARNING
            tooltip += '<dl><dt>Can\'t get node contact information, but there exists publisher, subscriber or services of this node.</dt></dl>'
            tooltip += '</dl>'
            self.setToolTip('<div>%s</div>' % tooltip)
        elif self.node_info.uri is not None:
            self._state = NodeItem.STATE_WARNING
            self.setIcon(nm.settings().icon('crystal_clear_warning.png'))
            if not self.node_info.isLocal and master_discovered:
                tooltip = '<h4>%s is not local, however the ROS master on this host is discovered, but no information about this node received!</h4>' % self.node_info.name
                self.setToolTip('<div>%s</div>' % tooltip)
        elif self.is_ghost:
            self._state = NodeItem.STATE_GHOST
            self.setIcon(nm.settings().icon('state_ghost.png'))
            tooltip = '<h4>The node is running, but not synchronized because of filter or errors, see master_sync log.</h4>'
            self.setToolTip('<div>%s</div>' % tooltip)
        elif self.has_running:
            self._state = NodeItem.STATE_DUPLICATE
            self.setIcon(nm.settings().icon('imacadam_stop.png'))
            tooltip = '<h4>There are nodes with the same name on remote hosts running. These will be terminated, if you run this node! (Only if master_sync is running or will be started somewhere!)</h4>'
            self.setToolTip('<div>%s</div>' % tooltip)
        else:
            self._state = NodeItem.STATE_OFF
            self.setIcon(nm.settings().icon('state_off.png'))
            self.setToolTip('')
        # removed common tooltip for clarity !!!
#    self.setToolTip(''.join(['<div>', tooltip, '</div>']))

    def update_displayed_url(self):
        '''
        Updates the URI representation in other column.
        '''
        if self.parent_item is not None:
            uri_col = self.parent_item.child(self.row(), NodeItem.COL_URI)
            if uri_col is not None and isinstance(uri_col, QStandardItem):
                uri_col.setText(utf8(self.node_info.uri) if self.node_info.uri is not None else "")

    def update_displayed_config(self):
        '''
        Updates the configuration representation in other column.
        '''
        if self.parent_item is not None:
            cfg_col = self.parent_item.child(self.row(), NodeItem.COL_CFG)
            if cfg_col is not None and isinstance(cfg_col, QStandardItem):
                cfg_count = len(self._cfgs)
                cfg_col.setText(utf8(''.join(['[', utf8(cfg_count), ']'])) if cfg_count > 1 else "")
                # no tooltip for clarity !!!
                # set icons
                has_launches = NodeItem.has_launch_cfgs(self._cfgs)
                has_defaults = NodeItem.has_default_cfgs(self._cfgs)
                if has_launches and has_defaults:
                    cfg_col.setIcon(nm.settings().icon('crystal_clear_launch_file_def_cfg.png'))
                elif has_launches:
                    cfg_col.setIcon(nm.settings().icon('crystal_clear_launch_file.png'))
                elif has_defaults:
                    cfg_col.setIcon(nm.settings().icon('default_cfg.png'))
                else:
                    cfg_col.setIcon(QIcon())
            # the update of the group will be perform in node_tree_model to reduce calls
            # if isinstance(self.parent_item, GroupItem):
            #     self.parent_item.update_displayed_config()

    @property
    def cfgs(self):
        '''
        Returns the list with all launch configurations assigned to this item.

        :rtype: list(str)
        '''
        return self._cfgs

    def add_config(self, cfg):
        '''
        Add the given configurations to the node.

        :param str cfg: the loaded configuration, which contains this node.
        '''
        if cfg == '':
            self._std_config = cfg
        if cfg and cfg not in self._cfgs:
            self._cfgs.append(cfg)
            self.update_displayed_config()

    def rem_config(self, cfg):
        '''
        Remove the given configurations from the node.

        :param str cfg: the loaded configuration, which contains this node.
        '''
        result = False
        if cfg == '':
            self._std_config = None
            result = True
        if cfg in self._cfgs:
            self._cfgs.remove(cfg)
            result = True
        if result and (self.has_configs() or self.is_running()):
            self.update_displayed_config()
        return result

    def readd(self):
        '''
        Remove this node from current group and put it in new one, defined by namespace.
        This is only executed if parent_item is valid and the name of this node has namespace.
        '''
        if self.parent_item is not None and self.with_namespace:
            row = None
            for i in reversed(range(self.parent_item.rowCount())):
                item = self.parent_item.child(i)
                if (type(item) == NodeItem) and item.name == self.name:
                    row = self.parent_item.takeRow(i)
                    break
            group_item = self.parent_item.get_group_item(namespace(item.name), is_group=False)
            group_item._add_row_sorted(row)
            group_item.updateIcon()

    def type(self):
        return NodeItem.ITEM_TYPE

    @classmethod
    def newNodeRow(self, name, masteruri):
        '''
        Creates a new node row and returns it as a list with items. This list is
        used for the visualization of node data as a table row.

        :param str name: the node name
        :param str masteruri: the URI or the ROS master assigned to this node.
        :return: the list for the representation as a row list(node name, configuration)
        :rtype: list(:class:`NodeItem`, :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        items = []
        item = NodeItem(NodeInfo(name, masteruri))
        items.append(item)
        cfgitem = CellItem(name, item)
        items.append(cfgitem)
#    uriitem = QStandardItem()
#    items.append(uriitem)
        return items

    def has_configs(self):
        return not (len(self._cfgs) == 0)

    def is_running(self):
        return not (self._node_info.pid is None and self._node_info.uri is None)

    def has_std_cfg(self):
        return self._std_config == ''

    def count_launch_cfgs(self):
        result = 0
        for c in self.cfgs:
            if not self.is_default_cfg(c):
                result += 1
        return result

    def count_default_cfgs(self):
        result = 0
        for c in self.cfgs:
            if self.is_default_cfg(c):
                result += 1
        return result

    @classmethod
    def has_launch_cfgs(cls, cfgs):
        for c in cfgs:
            if not cls.is_default_cfg(c):
                return True
        return False

    @classmethod
    def has_default_cfgs(cls, cfgs):
        for c in cfgs:
            if cls.is_default_cfg(c):
                return True
        return False

    @classmethod
    def is_default_cfg(cls, cfg):
        return isinstance(cfg, tuple)

    def __eq__(self, item):
        '''
        Compares the name of the node.
        '''
        if isstring(item):
            return self.name == item
        elif item is not None and type(item) == NodeItem:
            return self.name == item.name
        return False

    def __gt__(self, item):
        '''
        Compares the name of the node.
        '''
        if isstring(item):
            return self.name > item
        elif item is not None and type(item) == NodeItem:
            return self.name > item.name
        return False


# ###############################################################################
# #############                NodeTreeModel                       ##############
# ###############################################################################

class NodeTreeModel(QStandardItemModel):
    '''
    The model to show the nodes running in a ROS system or loaded by a launch
    configuration.
    '''
    header = [('Name', 450),
              ('Info', -1)]
#            ('URI', -1)]

    hostInserted = Signal(HostItem)
    ''':ivar HostItem hostInserted: the Qt signal, which is emitted, if a new host was inserted. Parameter: :class:`QtCore.QModelIndex` <https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html> of the inserted host item'''

    def __init__(self, host_address, masteruri, parent=None):
        '''
        Initialize the model.
        '''
        super(NodeTreeModel, self).__init__(parent)
        self.setColumnCount(len(NodeTreeModel.header))
        self.setHorizontalHeaderLabels([label for label, _ in NodeTreeModel.header])
        self._local_host_address = host_address
        self._local_masteruri = masteruri
        self._std_capabilities = {'/': {'SYSTEM': {'images': [],
                                                   'nodes': ['/rosout',
                                                             '/master_discovery',
                                                             '/zeroconf',
                                                             '/master_sync',
                                                             '/node_manager',
                                                             '/node_manager_daemon',
                                                             '/dynamic_reconfigure/*'],
                                                   'type': '',
                                                   'description': 'This group contains the system management nodes.'}}}

        # create a handler to request the parameter
        self.parameterHandler = ParameterHandler()
#    self.parameterHandler.parameter_list_signal.connect(self._on_param_list)
        self.parameterHandler.parameter_values_signal.connect(self._on_param_values)
#    self.parameterHandler.delivery_result_signal.connect(self._on_delivered_values)

    @property
    def local_addr(self):
        return self._local_host_address

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def _set_std_capabilities(self, host_item):
        if host_item is not None:
            cap = self._std_capabilities
            mastername = roslib.names.SEP.join(['', host_item.mastername, '*', 'default_cfg'])
            if mastername not in cap['/']['SYSTEM']['nodes']:
                cap['/']['SYSTEM']['nodes'].append(mastername)
            host_item.add_capabilities('', cap, host_item.masteruri)
            return cap
        return dict(self._std_capabilities)

    def get_hostitem(self, masteruri, address):
        '''
        Searches for the host item in the model. If no item is found a new one will
        created and inserted in sorted order.

        :param str masteruri: ROS master URI
        :param str address: the address of the host
        :return: the item associated with the given master
        :rtype: :class:`HostItem`
        '''
        if masteruri is None:
            return None
        master_entry = nm.nameres().get_master(masteruri, address)
        host = (masteruri, address)
        local = (self.local_addr in [address] + nm.nameres().resolve_cached(address) and
                 nmdurl.equal_uri(self._local_masteruri, masteruri))
        # find the host item by address
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            host_item = root.child(i)
            if host_item == master_entry:
                host_item.local = local
                return host_item
            elif host_item > host:
                items = []
                hostItem = HostItem(masteruri, address, local, master_entry)
                items.append(hostItem)
                cfgitem = CellItem(masteruri, hostItem)
                items.append(cfgitem)
                self.insertRow(i, items)
                self.hostInserted.emit(hostItem)
                self._set_std_capabilities(hostItem)
                return hostItem
        items = []
        hostItem = HostItem(masteruri, address, local, master_entry)
        items.append(hostItem)
        cfgitem = CellItem(masteruri, hostItem)
        items.append(cfgitem)
        self.appendRow(items)
        self.hostInserted.emit(hostItem)
        self._set_std_capabilities(hostItem)
        return hostItem

    def update_model_data(self, nodes, info_masteruri):
        '''
        Updates the model data.

        :param nodes: a dictionary with name and info objects of the nodes.
        :type nodes: dict(str: :class:`NodeInfo`)
        '''
        # separate into different hosts
        hosts = dict()
        muris = []
        addresses = []
        updated_nodes = []
        local_info = nmdurl.equal_uri(self._local_masteruri, info_masteruri)
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            host.reset_remote_launched_nodes()
        for (name, node) in nodes.items():
            addr = get_hostname(node.uri if node.uri is not None else node.masteruri)
            addresses.append(addr)
            muris.append(node.masteruri)
            host = self.get_hostitem(node.masteruri, addr)
            if host not in hosts:
                hosts[host] = dict()
            hosts[host][name] = node
        # update nodes for each host
        for (host_item, nodes_filtered) in hosts.items():
            # rename the host item if needed
            if host_item is not None:
                updated_nodes.extend(host_item.update_running_state(nodes_filtered, local_info))
            # request for all nodes in host the parameter capability_group
            self._requestCapabilityGroupParameter(host_item)
        # update nodes of the hosts, which are not more exists
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            # remove hosts if they are not updated
            if host.masteruri not in muris:
               host.update_running_state({}, local_info)
            # remove hosts which are connected to local master using ROS_MASTER_URI
            if (not host.local and host.host not in addresses):
                host.update_running_state({}, local_info)
        self._remove_empty_hosts()
        return updated_nodes
        # update the duplicate state
#    self.set_duplicate_nodes(self.get_nodes_running())

    def _requestCapabilityGroupParameter(self, host_item):
        if host_item is not None:
            items = host_item.get_node_items()
            params = [roslib.names.ns_join(item.name, 'capability_group') for item in items if not item.has_configs() and item.is_running() and not host_item.is_in_cap_group(item.name, '', '/', 'SYSTEM')]
            if params:
                self.parameterHandler.requestParameterValues(host_item.masteruri, params)

    def _on_param_values(self, masteruri, code, msg, params):
        '''
        Updates the capability groups of nodes from ROS parameter server.

        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param params: The dictionary of parameter names and request result as tuple(code, statusMessage, parameterValue)
        :type params: dict(str : (int, str, str)))
        '''
        host = get_hostname(masteruri)
        hostItem = self.get_hostitem(masteruri, host)
        changed = False
        if hostItem is not None and code == 1:
            capabilities = self._set_std_capabilities(hostItem)
            available_ns = set(['/'])
            available_groups = set(['SYSTEM'])
            # assumption: all parameter are 'capability_group' parameter
            for p, (code_n, _, val) in params.items():  # _:=msg_n
                nodename = roslib.names.namespace(p).rstrip(roslib.names.SEP)
                ns = roslib.names.namespace(nodename).rstrip(roslib.names.SEP)
                if not ns:
                    ns = roslib.names.SEP
                available_ns.add(ns)
                if code_n == 1:
                    # add group
                    if val:
                        available_groups.add(val)
                        if ns not in capabilities:
                            capabilities[ns] = dict()
                        if val not in capabilities[ns]:
                            capabilities[ns][val] = {'images': [], 'nodes': [], 'type': '', 'description': 'This group is created from `capability_group` parameter of the node defined in ROS parameter server.'}
                        if nodename not in capabilities[ns][val]['nodes']:
                            capabilities[ns][val]['nodes'].append(nodename)
                            changed = True
                else:
                    try:
                        for group, _ in list(capabilities[ns].items()):
                            try:
                                # remove the config from item, if parameter was not foun on the ROS parameter server
                                groupItem = hostItem.get_group_item(roslib.names.ns_join(ns, group), nocreate=True)
                                if groupItem is not None:
                                    nodeItems = groupItem.get_node_items_by_name(nodename, True)
                                    for item in nodeItems:
                                        item.rem_config('')
                                capabilities[ns][group]['nodes'].remove(nodename)
                                # remove the group, if empty
                                if not capabilities[ns][group]['nodes']:
                                    del capabilities[ns][group]
                                    if not capabilities[ns]:
                                        del capabilities[ns]
                                groupItem.update_displayed_config()
                                changed = True
                            except Exception:
                                pass
                    except Exception:
                        pass
            # clearup namespaces to remove empty groups
            for ns in list(capabilities.keys()):
                if ns and ns not in available_ns:
                    del capabilities[ns]
                    changed = True
                else:
                    for group in list(capabilities[ns].keys()):
                        if group and group not in available_groups:
                            del capabilities[ns][group]
                            changed = True
            # update the capabilities and the view
            if changed:
                if capabilities:
                    hostItem.add_capabilities('', capabilities, hostItem.masteruri)
            hostItem.clearup()
        else:
            rospy.logwarn("Error on retrieve \'capability group\' parameter from %s: %s", utf8(masteruri), msg)

    def update_system_diagnostics(self, masteruri, diagnostics):
        host = get_hostname(masteruri)
        host_item = self.get_hostitem(masteruri, host)
        if host_item is not None:
            host_item.update_system_diagnostics(diagnostics)

    def sysmon_set_state(self, masteruri, state):
        host = get_hostname(masteruri)
        host_item = self.get_hostitem(masteruri, host)
        if host_item is not None:
            host_item.sysmon_state = state

    def set_std_capablilities(self, capabilities):
        '''
        Sets the default capabilities description, which is assigned to each new
        host.

        :param capabilities: the structure for capabilities
        :type capabilities: {namespace: {group: {'type': str, 'description': str, 'nodes': [str]}}}
        '''
        self._std_capabilities = capabilities

    def add_capabilities(self, masteruri, host_address, cfg, capabilities):
        '''
        Adds groups to the model

        :param str masteruri: ROS master URI
        :param str host_address: the address the host
        :param str cfg: the configuration name (launch file name)
        :param capabilities: the structure for capabilities
        :type capabilities: {namespace: {group: {'type': str, 'description': str, 'nodes': [str]}}}
        '''
        hostItem = self.get_hostitem(masteruri, host_address)
        if hostItem is not None:
            # add new capabilities
            hostItem.add_capabilities(cfg, capabilities, hostItem.masteruri)
        self._remove_empty_hosts()

    def append_config(self, masteruri, host_address, nodes):
        '''
        Adds nodes to the model. If the node is already in the model, only his
        configuration list will be extended.

        :param str masteruri: ROS master URI
        :param str host_address: the address the host
        :param nodes: a dictionary with node names and their configurations
        :type nodes: {str: str}
        '''
        try:
            hostItem = self.get_hostitem(masteruri, host_address)
            if hostItem is not None:
                groups = {}
                cfgs = {}
                for (name, cfg) in nodes.items():
                    if cfg not in cfgs:
                        cfgs[cfg] = hostItem.get_node_items_by_cfg(cfg)
                    items = hostItem.get_node_items_by_name(name)
                    for item in items:
                        if item.parent_item is not None:
                            groups[item.parent_item.get_namespace()] = item.parent_item
                        item.add_config(cfg)
                        item.readd()
                        try:
                            cfgs[cfg].remove(item)
                        except Exception:
                            pass
                    if not items:
                        # create the new node
                        node_info = NodeInfo(name, masteruri)
                        hostItem.add_node(node_info, cfg)
                        # get the group of the added node to be able to update the group view, if needed
                        items = hostItem.get_node_items_by_name(name)
                        for item in items:
                            if item.parent_item is not None:
                                groups[item.parent_item.get_namespace()] = item.parent_item
                for cfg in cfgs:
                    for node in cfgs[cfg]:
                        node.rem_config(cfg)
                # update the changed groups
                for _name, g in groups.items():
                    g.update_displayed_config()
                hostItem.clearup()
            self._remove_empty_hosts()
        except Exception:
            rospy.logwarn('Error while apply configuration to current view: %s' %traceback.format_exc())
        # update the duplicate state
#    self.set_duplicate_nodes(self.get_nodes_running())

    def remove_config(self, cfg):
        '''
        Removes nodes from the model. If node is running or containing in other
        launch or default configurations , only his configuration list will be
        reduced.

        :param str cfg: the name of the confugration to close
        '''
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            items = host.get_node_items()
            groups = {}
            for item in items:
                removed = item.rem_config(cfg)
                if removed and item.parent_item is not None:
                    groups[item.parent_item.get_namespace()] = item.parent_item
            for _name, g in groups.items():
                g.update_displayed_config()
            host.rem_capablities(cfg)
            host.clearup()
            if host.rowCount() == 0:
                self.invisibleRootItem().removeRow(i)
            elif groups:
                # request for all nodes in host the parameter capability_group
                self._requestCapabilityGroupParameter(host)

    def _remove_empty_hosts(self):
        # remove empty hosts
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            if host.rowCount() == 0: # or not host.remote_launched_nodes_updated():
                self.invisibleRootItem().removeRow(i)

    def get_tree_node(self, node_name, masteruri):
        '''
        Since the same node can be included by different groups, this method searches
        for all nodes with given name and returns these items.

        :param str node_name: The name of the node
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            if host is not None and (masteruri is None or nmdurl.equal_uri(host.masteruri, masteruri)):
                res = host.get_node_items_by_name(node_name)
                if res:
                    return res
        return []

    def clear_multiple_screens(self, masteruri):
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            if host is not None and (masteruri is None or nmdurl.equal_uri(host.masteruri, masteruri)):
                host.clear_multiple_screens()

    def get_node_items_by_name(self, nodes, only_local=True):
        '''
        Returns a list with matched nodes.

        :rtype: list(str)
        '''
        result = list()
        # # determine all running nodes
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            if (only_local and host is not None and host.local) or not only_local:
                for node in nodes:
                    result[len(nodes):] = host.get_node_items_by_name(node)
        return result

    def get_nodes_running(self):
        '''
        Returns a list with all known running nodes.

        :rtype: list(str)
        '''
        running_nodes = list()
        # # determine all running nodes
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            if host is not None:  # should not occur
                running_nodes[len(running_nodes):] = host.get_nodes_running()
        return running_nodes

    def set_duplicate_nodes(self, running_nodes, is_sync_running=False):
        '''
        If there are a synchronization running, you have to avoid to running the
        node with the same name on different hosts. This method helps to find the
        nodes with same name running on other hosts and loaded by a configuration.
        The nodes loaded by a configuration will be inform about a currently running
        nodes, so a warning can be displayed!

        :param running_nodes: The dictionary with names of running nodes and their masteruri
        :type running_nodes: {str: str}
        :param bool is_sync_running: If the master_sync is running, the nodes are marked
          as ghost nodes. So they are handled as running nodes, but has not run
          informations. This nodes are running on remote host, but are not
          syncronized because of filter or errors.
        '''
        for i in reversed(range(self.invisibleRootItem().rowCount())):
            host = self.invisibleRootItem().child(i)
            if host is not None:  # should not occur
                host.set_duplicate_nodes(running_nodes, is_sync_running)

    def update_host_description(self, masteruri, host, descr_type, descr_name, descr):
        '''
        Updates the description of a host.

        :param str masteruri: ROS master URI of the host to update
        :param str host: host to update
        :param str descr_type: the type of the robot
        :param str descr_name: the name of the robot
        :param str descr: the description of the robot as a reStructuredText<http://docutils.sourceforge.net/rst.html>
        '''
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            if root.child(i) == (utf8(masteruri), utf8(host)):
                h = root.child(i)
                h.update_description(descr_type, descr_name, descr)
                return h.update_tooltip()


# ###############################################################################
# #############                NodeInfoIconsDelegate               ##############
# ###############################################################################

class NodeInfoIconsDelegate(QItemDelegate):
    '''
    Decorates the info column.
    '''

    def __init__(self, parent=None, *args):
        QItemDelegate.__init__(self, parent, *args)
        self._idx_icon = 1
        self._hspacing = 2
        self._vspacing = 2
        self._icon_size = 0
        self.IMAGES = {}

    def _scale_icons(self, icon_size):
        self._icon_size = icon_size
        params = (self._icon_size, self._icon_size, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        self.IMAGES = {'launchfile': nm.settings().image('crystal_clear_launch_file.png').scaled(*params),
                       'defaultcfg': nm.settings().image('default_cfg.png').scaled(*params),
                       'nodelet': nm.settings().image('crystal_clear_nodelet.png').scaled(*params),
                       'nodelet_mngr': nm.settings().image('crystal_clear_nodelet_mngr.png').scaled(*params),
                       'warning': nm.settings().image('crystal_clear_warning.png').scaled(*params),
                       'noscreen': nm.settings().image('crystal_clear_no_io.png').scaled(*params),
                       'misc': nm.settings().image('crystal_clear_miscellaneous.png').scaled(*params),
                       'group': nm.settings().image('crystal_clear_group.png').scaled(*params),
                       'mscreens': nm.settings().image('crystal_clear_mscreens.png').scaled(*params),
                       'sysmon': nm.settings().image('crystal_clear_get_parameter.png').scaled(*params),
                       'clock_warn': nm.settings().image('crystal_clear_xclock_fail.png').scaled(*params),
                       'cpu_warn': nm.settings().image('hight_load.png').scaled(*params),
                       'cpu_temp_warn': nm.settings().image('temperatur_warn.png').scaled(*params),
                       'hdd_warn': nm.settings().image('crystal_clear_hdd_warn.png').scaled(*params),
                       'net_warn': nm.settings().image('sekkyumu_net_warn.png').scaled(*params),
                       'mem_warn': nm.settings().image('mem_warn.png').scaled(*params)
                       }

    def paint(self, painter, option, index):
        if option.rect.height() - self._vspacing * 2 != self._icon_size:
            self._icon_size = option.rect.height() - self._vspacing * 2
            self._scale_icons(self._icon_size)
        painter.save()
        self._idx_icon = 1
        # we assume the model has an filter proxy installed
        model_index = index.model().mapToSource(index)
        item = model_index.model().itemFromIndex(model_index)
        if isinstance(item, CellItem):
            if isinstance(item.item, NodeItem):
                tooltip = ''
                if item.item.has_multiple_screens:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['mscreens'])
                    tooltip += 'multiple screens'
                if not item.item.has_screen:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['noscreen'])
                    tooltip += 'no screen'
                lcfgs = item.item.count_launch_cfgs()
                if lcfgs > 0:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['launchfile'])
                    if lcfgs > 1:
                        painter.drawText(rect, Qt.AlignCenter, str(lcfgs))
                if item.item.nodelets:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['nodelet_mngr'])
                if item.item.nodelet_mngr:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['nodelet'])
                if item.item.nodelets:
                    tooltip += "%sThis is a nodelet manager" % '\n' if tooltip else ''
                elif item.item.nodelet_mngr:
                    tooltip += "%sThis is a nodelet for %s" % ('\n' if tooltip else '', item.item.nodelet_mngr)
                item.setToolTip(tooltip)
            elif isinstance(item.item, HostItem):
                tooltip = ''
                if item.item.sysmon_state:
                    tooltip += '<dt><font color="orange">%s</font></dt>' % ("active pull for system diagnostic is enabled")
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['sysmon'])
                diagnistics = item.item.diagnostics
                for diag in diagnistics:
                    if diag.level > 0:
                        tooltip += '\n<dt><font color="red">%s</font></dt>' % (diag.message.replace('>', '&gt;').replace('<', '&lt;'))
                        if 'Network Load' in diag.name:
                            rect = self.calcDecorationRect(option.rect)
                            painter.drawImage(rect, self.IMAGES['net_warn'])
                        if 'CPU Load' in diag.name:
                            rect = self.calcDecorationRect(option.rect)
                            painter.drawImage(rect, self.IMAGES['cpu_warn'])
                        if 'CPU Temperature' in diag.name:
                            rect = self.calcDecorationRect(option.rect)
                            painter.drawImage(rect, self.IMAGES['cpu_temp_warn'])
                        if 'Memory Usage' in diag.name:
                            rect = self.calcDecorationRect(option.rect)
                            painter.drawImage(rect, self.IMAGES['mem_warn'])
                        if 'HDD Usage' in diag.name:
                            rect = self.calcDecorationRect(option.rect)
                            painter.drawImage(rect, self.IMAGES['hdd_warn'])
                item.setToolTip(tooltip)
            elif isinstance(item.item, GroupItem):
                lcfgs = len(item.item.get_configs())
                rect = self.calcDecorationRect(option.rect)
                painter.drawImage(rect, self.IMAGES['group'])
                count_nodes = item.item.count_nodes()
                if count_nodes > 1:
                    painter.drawText(rect, Qt.AlignCenter, str(count_nodes))
                if lcfgs > 0:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['launchfile'])
                    if lcfgs > 1:
                        painter.drawText(rect, Qt.AlignCenter, str(lcfgs))
                mscrens = item.item.get_count_mscreens()
                if mscrens > 0:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['mscreens'])
#                    if mscrens > 1:
#                        painter.drawText(rect, Qt.AlignCenter, str(mscrens))
        painter.restore()

    def calcDecorationRect(self, main_rect, image=True):
        rect = QRect()
        rect.setX(main_rect.x() + self._idx_icon + self._hspacing)
        rect.setY(main_rect.y() + self._vspacing)
        rect.setWidth(self._icon_size if image else main_rect.width() - self._idx_icon)
        rect.setHeight(self._icon_size)
        self._idx_icon += self._icon_size + self._hspacing
        return rect
