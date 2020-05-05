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



import re
import rospy
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QIcon, QStandardItem, QStandardItemModel

from fkie_master_discovery.master_info import TopicInfo

from fkie_node_manager_daemon.common import isstring, utf8
from fkie_node_manager.common import lnamespace, namespace, normns
from fkie_node_manager.detailed_msg_box import MessageBox
import fkie_node_manager as nm


class TopicItem(QStandardItem):
    '''
    The topic item stored in the topic model. This class stores the topic as
    U{fkie_master_discovery.TopicInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.TopicInfo>}.
    The name of the topic represented in HTML.
    '''

    ITEM_TYPE = QStandardItem.UserType + 36
    NAME_ROLE = Qt.UserRole + 1
    NODENAMES_ROLE = Qt.UserRole + 2
    COL_PUB = 1
    COL_SUB = 2
    COL_TYPE = 3

    def __init__(self, name, topic=None, parent=None):
        '''
        Initialize the topic item.
        :param str name: the topic name
        :param topic: the topic info
        :type topic: U{fkie_master_discovery.TopicInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.TopicInfo>}
        '''
        QStandardItem.__init__(self, name)
        self._parent_item = parent
        self._publish_thread = None
        self.topic = TopicInfo(name) if topic is None else topic
        '''@ivar: topic as U{fkie_master_discovery.TopicInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.TopicInfo>}.'''
        self._with_namespace = rospy.names.SEP in name

#  def __del__(self):
#    print "delete TOPIC", self.__topic.name

    @property
    def name(self):
        return self.text()

    @name.setter
    def name(self, new_name):
        self.setText(new_name)

    @property
    def topic_type_str(self):
        return self.topic.type

    @property
    def parent_item(self):
        return self._parent_item

    @parent_item.setter
    def parent_item(self, parent_item):
        self._parent_item = parent_item
        if parent_item is None:
            self.setText(self.topic.name)
            self._with_namespace = rospy.names.SEP in self.text()
        else:
            new_name = self.topic.name.replace(parent_item.get_namespace(), '', 1)
            self.setText(new_name)
            self._with_namespace = rospy.names.SEP in new_name

    @property
    def with_namespace(self):
        '''
        Returns `True` if the topic name contains a '/' in his name

        :rtype: bool
        '''
        return self._with_namespace

    def update_view(self, topic_info=None):
        '''
        Updates the view
        '''
        if topic_info is not None:
            self.topic = topic_info
        self.updatePublisherView()
        self.updateSubscriberView()
        self.updateTypeView()

    def updatePublisherView(self):
        '''
        Updates the representation of the column contains the publisher state.
        '''
        if self.parent_item is not None:
            cfg_col = self.parent_item.child(self.row(), TopicItem.COL_PUB)
            if cfg_col is not None and isinstance(cfg_col, QStandardItem):
                cfg_col.setText(str(len(self.topic.publisherNodes)))
                tooltip = ''.join(['<h4>', 'Publisher [', self.topic.name, ']:</h4><dl>'])
                for p in self.topic.publisherNodes:
                    tooltip = ''.join([tooltip, '<dt>', p, '</dt>'])
                tooltip = ''.join([tooltip, '</dl>'])
                if len(self.topic.publisherNodes) > 0:
                    cfg_col.setToolTip(''.join(['<div>', tooltip, '</div>']))

    def updateSubscriberView(self):
        '''
        Updates the representation of the column contains the subscriber state.
        '''
        if self.parent_item is not None:
            cfg_col = self.parent_item.child(self.row(), TopicItem.COL_SUB)
            if cfg_col is not None and isinstance(cfg_col, QStandardItem):
                cfg_col.setText(str(len(self.topic.subscriberNodes)))
                tooltip = ''.join(['<h4>', 'Subscriber [', self.topic.name, ']:</h4><dl>'])
                for p in self.topic.subscriberNodes:
                    tooltip = ''.join([tooltip, '<dt>', p, '</dt>'])
                tooltip = ''.join([tooltip, '</dl>'])
                if len(self.topic.subscriberNodes) > 0:
                    cfg_col.setToolTip(''.join(['<div>', tooltip, '</div>']))

    def updateTypeView(self):
        '''
        Updates the representation of the column contains the type of the topic.
        '''
        if self.parent_item is not None:
            cfg_col = self.parent_item.child(self.row(), TopicItem.COL_TYPE)
            if cfg_col is not None and isinstance(cfg_col, QStandardItem):
                cfg_col.setText(self.topic.type if self.topic.type and self.topic.type != 'None' else 'unknown type')
                # removed tooltip for clarity!!!
#         if not self.topic.type is None and not cfg_col.toolTip():
#           return
#           # removed tooltip for clarity !!!
# #          tooltip = ''
#           try:
#             mclass = roslib.message.get_message_class(self.topic.type)
# #            tooltip = utf8(mclass)
#             if not mclass is None:
# #              tooltip = utf8(mclass.__slots__)
#               for f in mclass.__slots__:
#                 idx = mclass.__slots__.index(f)
#                 idtype = mclass._slot_types[idx]
#                 base_type = roslib.msgs.base_msg_type(idtype)
#                 primitive = "unknown"
#                 if base_type in roslib.msgs.PRIMITIVE_TYPES:
#                   primitive = "primitive"
#                 else:
#                   try:
#                     list_msg_class = roslib.message.get_message_class(base_type)
#                     primitive = "class", list_msg_class.__slots__
#                   except ValueError:
#                     pass
# #                tooltip = ''.join([tooltip, '\n\t', utf8(f), ': ', utf8(idtype), ' (', utf8(primitive),')'])
#           except ValueError:
#             pass
#          cfg_col.setToolTip(tooltip)

    def _on_wait_for_publishing(self):
        self.updateIconView(nm.settings().icon('state_off.png'))

    def _on_partial_publishing(self):
        self.updateIconView(nm.settings().icon('state_part.png'))

    def _on_publishing(self):
        self.updateIconView(nm.settings().icon('state_run.png'))

    def _publish_finished(self):
        self._publish_thread = None
        self.setIcon(QIcon())

    def show_error_msg(self, msg):
        MessageBox.warning(self, "Publish error",
                           'Error while publish to %s' % self.topic.name,
                           tr(utf8(msg)))

    def type(self):
        return TopicItem.ITEM_TYPE

    def data(self, role):
        if role == self.NAME_ROLE:
            return self.topic.name
        elif role == self.NODENAMES_ROLE:
            return utf8(self.topic.publisherNodes) + utf8(self.topic.subscriberNodes)
        else:
            return QStandardItem.data(self, role)

    @classmethod
    def create_item_list(self, topic, root):
        '''
        Creates the list of the items from topic. This list is used for the
        visualization of topic data as a table row.
        :param str topic: the topic name
        :param root: The parent QStandardItem
        :type root: U{QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}
        :return: the list for the representation as a row
        :rtype: C{[L{TopicItem} or U{QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}, ...]}
        '''
        items = []
        item = TopicItem(topic.name, topic, parent=root)
        items.append(item)
        pubItem = QStandardItem()
#    TopicItem.updatePublisherView(topic, pubItem)
        items.append(pubItem)
        subItem = QStandardItem()
#    TopicItem.updateSubscriberView(topic, subItem)
        items.append(subItem)
        typeItem = QStandardItem()
#    TopicItem.updateTypeView(topic, typeItem)
        items.append(typeItem)
        return items

    def __eq__(self, item):
        '''
          Compares the name of topic.
          '''
        if isstring(item):
            return self.topic.name.lower() == item.lower()
        elif not (item is None):
            return self.topic.name.lower() == item.topic.name.lower()
        return False


# ###############################################################################
# #############                  GrouptItem                        ##############
# ###############################################################################
class TopicGroupItem(QStandardItem):
    '''
    The TopicGroupItem stores the information about a group of nodes.
    '''
    ITEM_TYPE = Qt.UserRole + 35

    def __init__(self, name, parent=None, is_group=False):
        '''
        Initialize the TopicGroupItem object with given values.

        :param str name: the name of the group
        :param parent: the parent item. In most cases this is the HostItem. The variable is used to determine the different columns of the NodeItem.
        :type parent: :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>
        :param bool is_group: True if this is a capability group. In other case it is a namespace group.
        '''
        dname = 'topics@master/'
        if is_group:
            dname = '{%s}' % name
        elif name != rospy.names.SEP:
            dname = '%s/' % name
        QStandardItem.__init__(self, dname)
        self.parent_item = parent
        self._name = name
        self._is_group = is_group
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
    def is_group(self):
        return self._is_group

    def get_namespace(self):
        name = self._name
        if type(self) == TopicGroupItem and self._is_group:
            name = namespace(self._name)
        result = name
        if self.parent_item is not None and type(self.parent_item) != QStandardItem:
            result = normns(self.parent_item.get_namespace() + rospy.names.SEP) + normns(result + rospy.names.SEP)
        return normns(result)

    def count_topics(self):
        '''
        :retrun: Returns count of nodes inside this group.
        :rtype: int
        '''
        result = 0
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, TopicGroupItem):
                result += item.count_nodes()
            elif isinstance(item, TopicItem):
                result += 1
        return result

    def get_topic_items_by_name(self, topic_name, recursive=True):
        '''
        Since the same node can be included by different groups, this method searches
        for all nodes with given name and returns these items.

        :param str topic_name: The name of the topic
        :param bool recursive: Searches in (sub) groups
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, TopicGroupItem):
                if recursive:
                    result[len(result):] = item.get_topic_items_by_name(topic_name)
            elif isinstance(item, TopicItem) and item == topic_name:
                return [item]
        return result

    def get_topic_items(self, recursive=True):
        '''
        Returns all nodes in this group and subgroups.

        :param bool recursive: returns the nodes of the subgroups
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, TopicGroupItem):
                if recursive:
                    result[len(result):] = item.get_topic_items()
            elif isinstance(item, TopicItem):
                result.append(item)
        return result

    @classmethod
    def create_item_list(self, name, parent, is_group):
        '''
        Creates the list of the items for this group. This list is used for the
        visualization of group data as a table row.

        :param str name: the group name
        :return: the list for the representation as a row
        :rtype: C{[L{TopicGroupItem} or U{QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}, ...]}
        '''
        items = []
        item = TopicGroupItem(name, parent, is_group)
        items.append(item)
        pubItem = QStandardItem()
        items.append(pubItem)
        subItem = QStandardItem()
        items.append(subItem)
        typeItem = QStandardItem()
        items.append(typeItem)
        return items

    def get_group_item(self, group_name, is_group=True, nocreate=False):
        '''
        Returns a TopicGroupItem with given name. If no group with this name exists, a
        new one will be created. The given name will be split by slashes if exists
        and subgroups are created.

        :param str group_name: the name of the group
        :param bool is_group: True if it is a capability group. False if a namespace group. (Default: True)
        :param bool nocreate: avoid creation of new group if not exists. (Default: False)
        :return: The group with given name of None if `nocreate` is True and group not exists.
        :rtype: :class:`TopicGroupItem`
        '''
        lns, rns = group_name, ''
        if nm.settings().group_nodes_by_namespace:
            lns, rns = lnamespace(group_name)
            if lns == rospy.names.SEP:
                lns, rns = lnamespace(rns)
        if lns == rospy.names.SEP:
            return self
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, TopicGroupItem):
                if item == lns:
                    if rns:
                        return item.get_group_item(rns, is_group)
                    return item
                elif item > lns and not nocreate:
                    items = TopicGroupItem.create_item_list(lns, self, is_group=(is_group and not rns))
                    self.insertRow(i, items)
                    if rns:
                        return items[0].get_group_item(rns, is_group)
                    return items[0]
        if nocreate:
            return None
        items = TopicGroupItem.create_item_list(lns, self, is_group=(is_group and not rns))
        self.appendRow(items)
        if rns:
            return items[0].get_group_item(rns, is_group)
        return items[0]

    def add_node(self, topic):
        '''
        Adds a new topic with given name.

        :param topic: the TopicInfo of the node to create
        :type topic: :class:`TopicInfo`
        '''
        group_item = self
        if nm.settings().group_nodes_by_namespace:
            ns = namespace(topic.name)
            if ns != rospy.names.SEP:
                # insert in the group
                group_item = self.get_group_item(ns, False)
        # append new topic row
        new_item_row = TopicItem.create_item_list(topic, self)
        group_item._add_row(new_item_row)

    def _add_row(self, row):
        self.appendRow(row)
        row[0].parent_item = self
        row[0].update_view()

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
            if isinstance(item, TopicItem):
                pass
            else:  # if type(item) == TopicGroupItem:
                removed = item._clearup(fixed_node_names) or removed
        if self.rowCount() == 0 and self.parent_item is not None:
            self.parent_item._remove_group(self.name)
        return removed

    def _remove_group(self, name):
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == TopicGroupItem and item == name and item.rowCount() == 0:
                self.removeRow(i)
                return  # we assume only one group with same name can exists

    def _mark_groups_to_delete(self):
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, TopicItem):
                # remove group if only one node is inside
                if self.rowCount() == 1:
                    if not self.is_group:
                        if self.parent_item is not None:
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
            if isinstance(item, TopicGroupItem):
                if item._clearup_mark_delete:
                    row = self._take_node_row(item)
                    if row is not None:
                        rows2add.append(row)
                        self.removeRow(i)
                else:
                    item._remove_marked_groups()
        for row in rows2add:
            self._add_row(row)

    def _take_node_row(self, group):
        result = None
        if group.rowCount() == 1:
            item = group.child(0)
            if isinstance(item, TopicItem):
                result = group.takeRow(0)
            else:
                result = group._take_node_row(item)
        return result

    def remove_node(self, name):
        removed = False
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == TopicItem and item == name:
                self.removeRow(i)
                removed = True
                break
            elif type(item) == TopicGroupItem:
                removed = item.remove_node(name)
                if removed:
                    break
        if removed and self.rowCount() == 0:
            if type(self.parent_item) == TopicGroupItem:
                self.parent_item._remove_group(self.name)
        return removed

    def update_topic_view(self, updated_topics, topics):
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == TopicItem:
                if item.topic.name in updated_topics:
                    item.update_view(topics[item.topic.name])
            elif type(item) == TopicGroupItem:
                item.update_topic_view(updated_topics, topics)

    def index_from_names(self, publisher, subscriber):
        '''
        Returns for given topics the list of QModelIndex in this model.

        :param [str] publisher: the list of publisher topics
        :param [str] subscriber: the list of subscriber topics
        :return: the list of QModelIndex
        :rtype: [QtCore.QModelIndex]
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == TopicGroupItem:
                result[len(result):] = item.index_from_names(publisher, subscriber)
            elif type(item) == TopicItem:
                if item.topic.name in publisher:
                    result.append(item.index())
                    result.append(self.child(i, 1).index())  # select also the publishers column
                if item.topic.name in subscriber:
                    result.append(item.index())
                    result.append(self.child(i, 2).index())  # select also the subscribers column
        return result

    def type(self):
        return TopicGroupItem.ITEM_TYPE

    def __eq__(self, item):
        '''
        Compares the name of the group.
        '''
        if isstring(item):
            return self.name.lower() == item.lower()
        elif not (item is None):
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
        elif not (item is None):
            # put the group with SYSTEM nodes at the end
            if item.is_system_group:
                if self.name.lower() != item.lower():
                    return True
            elif self.is_syste_group:
                return False
            return self.name.lower() > item.name.lower()
        return False


class TopicModel(QStandardItemModel):
    '''
    The model to manage the list with topics in ROS network.
    '''
    header = [('Name', 300),
              ('Publisher', 50),
              ('Subscriber', 50),
              ('Type', -1)]
    ''':ivar: the list with columns C{[(name, width), ...]}'''

    def __init__(self):
        '''
        Creates a new list model.
        '''
        QStandardItemModel.__init__(self)
        self.setColumnCount(len(TopicModel.header))
        self.setHorizontalHeaderLabels([label for label, _ in TopicModel.header])
        topics = ['/rosout', '/rosout_agg', '/diagnostics', '/diagnostics_agg']
        def_list = ['\A' + n.strip().replace('*', '.*') + '\Z' for n in topics]
        self._re_cap_systopics = re.compile('|'.join(def_list), re.I)
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
        root_items = TopicGroupItem.create_item_list(rospy.names.SEP, self.invisibleRootItem(), False)
        self.invisibleRootItem().appendRow(root_items)
        self._pyqt_workaround_add(rospy.names.SEP, root_items[0])

    def flags(self, index):
        '''
        :param index: parent of the list
        :type index: QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>
        :return: Flag or the requested item
        :rtype: QtCore.Qt.ItemFlag<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>
        :see: http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html
        '''
        if not index.isValid():
            return Qt.NoItemFlags
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def get_cap_group(self, topic_name):
        match = False
        try:
            match = self._re_cap_systopics.match(topic_name)
        except Exception:
            pass
        if match:
            root = self.get_root_group()
            for i in range(root.rowCount()):
                item = root.child(i)
                if type(item) == TopicGroupItem:
                    if item == 'SYSTEM' and item.is_group:
                        return item
            items = TopicGroupItem.create_item_list('SYSTEM', root, True)
            root.appendRow(items)
            self.pyqt_workaround['{SYSTEM}'] = items[0]
            return items[0]
        return None

    def get_root_group(self):
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == TopicGroupItem:
                if item == rospy.names.SEP:
                    return item
        return None

    def update_model_data(self, topics, added_topics, updated_topics, removed_topics):
        '''
        Updates the topics model. New topic will be inserted in sorting order. Not
        available topics removed from the model.

        :param topics: The dictionary with topics
        :type topics: {topic name : U{fkie_master_discovery.TopicInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.TopicInfo>}}
        :param added_topics: the list of new topics in the :topics: list
        :type added_topics: list or set
        :param updated_topics: the list of updated topics in the :topics: list
        :type updated_topics: list or set
        :param removed_topics: the list of removed topics in the :topics: list
        :type removed_topics: list or set
        '''
        # first: remove topics
        for rm_topic in removed_topics:
            self._remove_node(rm_topic)
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            item = root.child(i)
            if type(item) == TopicGroupItem:
                item.update_topic_view(updated_topics, topics)
#    cputimes = os.times()
#    cputime_init = cputimes[0] + cputimes[1]
        # insert other items in sorted order
        # last: add new topics
        for topic_name in added_topics:
            try:
                topic = topics[topic_name]
                # first: add to system group
                sys_group = self.get_cap_group(topic_name)
                if sys_group is not None:
                    sys_group.add_node(topic)
                else:
                    # second add to the root group
                    root_group = self.get_root_group()
                    if root_group is not None:
                        root_group.add_node(topic)
            except Exception:
                import traceback
                print(traceback.format_exc())
        # remove empty groups
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == TopicGroupItem:
                item.clearup()
#    cputimes = os.times()
#    cputime = cputimes[0] + cputimes[1] - cputime_init
#    print "      update topic ", cputime, ", topic count:", len(topics)

    def index_from_names(self, publisher, subscriber):
        '''
        Returns for given topics the list of QModelIndex in this model.

        :param [str] publisher: the list of publisher topics
        :param [str] subscriber: the list of subscriber topics
        :return: the list of QModelIndex
        :rtype: [QtCore.QModelIndex]
        '''
        result = []
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == TopicGroupItem:
                result[len(result):] = item.index_from_names(publisher, subscriber)
            elif type(item) == TopicItem:
                if item.topic.name in publisher:
                    result.append(item.index())
                    result.append(self.child(i, 1).index())  # select also the publishers column
                if item.topic.name in subscriber:
                    result.append(item.index())
                    result.append(self.child(i, 2).index())  # select also the subscribers column
        return result

    def _remove_node(self, name):
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == TopicGroupItem:
                removed = item.remove_node(name)
                if removed:
                    return item
        return None

    def _pyqt_workaround_add(self, name, item):
        self.pyqt_workaround[name] = item  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass

    def _pyqt_workaround_rem(self, name):
        try:
            del self.pyqt_workaround[name]  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
        except Exception:
            pass
