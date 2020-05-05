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
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel

from fkie_master_discovery.common import get_hostname
from fkie_node_manager_daemon.common import isstring, utf8
from fkie_node_manager.common import lnamespace, namespace, normns
import fkie_node_manager as nm


class ServiceItem(QStandardItem):
    '''
    The service item stored in the service model. This class stores the service as
    U{fkie_master_discovery.ServiceInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.ServiceInfo>}.
    The name of the service is represented in HTML.
    '''

    ITEM_TYPE = QStandardItem.UserType + 37
    NAME_ROLE = Qt.UserRole + 1
    TYPE_ROLE = Qt.UserRole + 2
    NODENAMES_ROLE = Qt.UserRole + 3

    def __init__(self, service, parent=None):
        '''
        Initialize the service item.

        :param service: the service object to view
        :type service: fkie_master_discovery.ServiceInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.ServiceInfo>
        '''
        QStandardItem.__init__(self, service.name)
        self._parent_item = parent
        self.service = service
        ''':ivar self.service: service info as :ref:`fkie_master_discovery.ServiceInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.ServiceInfo>`.'''
        self._with_namespace = rospy.names.SEP in service.name

    @property
    def name(self):
        return self.text()

    @name.setter
    def name(self, new_name):
        self.setText(new_name)

    @property
    def service_type_str(self):
        stype = ''
        try:
            stype = utf8(self.service.get_service_class(False))
        except Exception:
            pass
        return stype

    @property
    def parent_item(self):
        return self._parent_item

    @parent_item.setter
    def parent_item(self, parent_item):
        self._parent_item = parent_item
        if parent_item is None:
            self.setText(self.service.name)
            self._with_namespace = rospy.names.SEP in self.text()
        else:
            new_name = self.service.name.replace(parent_item.get_namespace(), '', 1)
            self.setText(new_name)
            self._with_namespace = rospy.names.SEP in new_name

    @property
    def with_namespace(self):
        '''
        Returns `True` if the topic name contains a '/' in his name

        :rtype: bool
        '''
        return self._with_namespace

    def update_view(self, service_info=None):
        '''
        Updates the view
        '''
        if service_info is not None:
            self.service = service_info
        self.updateServiceView()

    def updateServiceView(self):
        '''
        Updates the view of the service on changes.

        :param parent: the item containing this item
        :type parent: :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>
        '''
        if self.parent_item is not None:
            # update type view
            child = self.parent_item.child(self.row(), 1)
            if child is not None:
                self.updateTypeView(self.service, child)

    def type(self):
        return ServiceItem.ITEM_TYPE

    @classmethod
    def create_item_list(self, service, root):
        '''
        Creates the list of the items from service. This list is used for the
        visualization of service data as a table row.

        :param service: the service data
        :type service: U{fkie_master_discovery.ServiceInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.ServiceInfo>}
        :return: the list for the representation as a row
        :rtype: [:class:`ServiceItem` or :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>]
        '''
        items = []
        item = ServiceItem(service, parent=root)
        # removed tooltip for clarity !!!
#    item.setToolTip(''.join(['<div><h4>', utf8(service.name), '</h4><dl><dt>', utf8(service.uri),'</dt></dl></div>']))
        items.append(item)
        typeItem = QStandardItem()
        ServiceItem.updateTypeView(service, typeItem)
        items.append(typeItem)
        return items

    @classmethod
    def updateTypeView(cls, service, item):
        '''
        Updates the representation of the column contains the type of the service.

        :param service: the service data
        :type service: fkie_master_discovery.ServiceInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.ServiceInfo>
        :param item: corresponding item in the model
        :type item: :class:`ServiceItem`
        '''
        try:
            if service.isLocal and service.type:
                service_class = service.get_service_class(nm.is_local(get_hostname(service.uri)))
                item.setText(service_class._type)
            elif service.type:
                item.setText(service.type)
            else:
                item.setText('unknown type')
            # removed tooltip for clarity !!!
#      tooltip = ''
#      tooltip = ''.join([tooltip, '<h4>', service_class._type, '</h4>'])
#      tooltip = ''.join([tooltip, '<b><u>', 'Request', ':</u></b>'])
#      tooltip = ''.join([tooltip, '<dl><dt>', utf8(service_class._request_class.__slots__), '</dt></dl>'])
#
#      tooltip = ''.join([tooltip, '<b><u>', 'Response', ':</u></b>'])
#      tooltip = ''.join([tooltip, '<dl><dt>', utf8(service_class._response_class.__slots__), '</dt></dl>'])
#
#      item.setToolTip(''.join(['<div>', tooltip, '</div>']))
            item.setToolTip('')
        except Exception:
            if not service.isLocal:
                tooltip = ''.join(['<h4>', 'Service type is not available due to he running on another host.', '</h4>'])
                item.setToolTip(''.join(['<div>', tooltip, '</div>']))

    def data(self, role):
        if role == self.NAME_ROLE:
            return self.service.name
        elif role == self.TYPE_ROLE:
            return self.service_type_str
        elif role == self.NODENAMES_ROLE:
            return utf8(self.service.serviceProvider)
        else:
            return QStandardItem.data(self, role)

    def __eq__(self, item):
        '''
        Compares the name of service.
        '''
        if isstring(item):
            return self.service.name.lower() == item.lower()
        elif not (item is None):
            return self.service.name.lower() == item.service.name.lower()
        return False


# ###############################################################################
# #############                  GrouptItem                        ##############
# ###############################################################################
class ServiceGroupItem(QStandardItem):
    '''
    The ServiceGroupItem stores the information about a group of nodes.
    '''
    ITEM_TYPE = Qt.UserRole + 45

    def __init__(self, name, parent=None, is_group=False):
        '''
        Initialize the ServiceGroupItem object with given values.

        :param str name: the name of the group
        :param parent: the parent item. In most cases this is the HostItem. The variable is used to determine the different columns of the NodeItem.
        :type parent: :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>
        :param bool is_group: True if this is a capability group. In other case it is a namespace group.
        '''
        dname = 'services@master/'
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
        if type(self) == ServiceGroupItem and self._is_group:
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
            if isinstance(item, ServiceGroupItem):
                result += item.count_nodes()
            elif isinstance(item, ServiceItem):
                result += 1
        return result

    def get_service_items_by_name(self, name, recursive=True):
        '''
        Since the same node can be included by different groups, this method searches
        for all nodes with given name and returns these items.

        :param str name: The name of the topic
        :param bool recursive: Searches in (sub) groups
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, ServiceGroupItem):
                if recursive:
                    result[len(result):] = item.get_service_items_by_name(name)
            elif isinstance(item, ServiceItem) and item == name:
                return [item]
        return result

    def get_service_items(self, recursive=True):
        '''
        Returns all nodes in this group and subgroups.

        :param bool recursive: returns the nodes of the subgroups
        :return: The list with node items.
        :rtype: list(:class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>)
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, ServiceGroupItem):
                if recursive:
                    result[len(result):] = item.get_service_items()
            elif isinstance(item, ServiceItem):
                result.append(item)
        return result

    @classmethod
    def create_item_list(self, name, parent, is_group):
        '''
        Creates the list of the items for this group. This list is used for the
        visualization of group data as a table row.

        :param str name: the group name
        :return: the list for the representation as a row
        :rtype: [:class:`ServiceGroupItem` or :class:`QtGui.QStandardItem` <https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}]
        '''
        items = []
        item = ServiceGroupItem(name, parent, is_group)
        items.append(item)
        typeItem = QStandardItem()
        items.append(typeItem)
        return items

    def get_group_item(self, group_name, is_group=True, nocreate=False):
        '''
        Returns a ServiceGroupItem with given name. If no group with this name exists, a
        new one will be created. The given name will be split by slashes if exists
        and subgroups are created.

        :param str group_name: the name of the group
        :param bool is_group: True if it is a capability group. False if a namespace group. (Default: True)
        :param bool nocreate: avoid creation of new group if not exists. (Default: False)
        :return: The group with given name of None if `nocreate` is True and group not exists.
        :rtype: :class:`ServiceGroupItem`
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
            if isinstance(item, ServiceGroupItem):
                if item == lns:
                    if rns:
                        return item.get_group_item(rns, is_group)
                    return item
                elif item > lns and not nocreate:
                    items = ServiceGroupItem.create_item_list(lns, self, is_group=(is_group and not rns))
                    self.insertRow(i, items)
                    if rns:
                        return items[0].get_group_item(rns, is_group)
                    return items[0]
        if nocreate:
            return None
        items = ServiceGroupItem.create_item_list(lns, self, is_group=(is_group and not rns))
        self.appendRow(items)
        if rns:
            return items[0].get_group_item(rns, is_group)
        return items[0]

    def add_node(self, service):
        '''
        Adds a new topic with given name.

        :param service: the TopicInfo of the node to create
        :type service: :class:`TopicInfo`
        '''
        group_item = self
        if nm.settings().group_nodes_by_namespace:
            ns = namespace(service.name)
            if ns != rospy.names.SEP:
                # insert in the group
                group_item = self.get_group_item(ns, False)
        # append new topic row
        new_item_row = ServiceItem.create_item_list(service, self)
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
            if isinstance(item, ServiceItem):
                pass
            else:  # if type(item) == ServiceGroupItem:
                removed = item._clearup(fixed_node_names) or removed
        if self.rowCount() == 0 and self.parent_item is not None:
            self.parent_item._remove_group(self.name)
        return removed

    def _remove_group(self, name):
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == ServiceGroupItem and item == name and item.rowCount() == 0:
                self.removeRow(i)
                return  # we assume only one group with same name can exists

    def _mark_groups_to_delete(self):
        for i in range(self.rowCount()):
            item = self.child(i)
            if isinstance(item, ServiceItem):
                # remove group if only one node is inside
                if self.rowCount() == 1:
                    if not self.is_group and not type(self) == QStandardItem:
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
            if isinstance(item, ServiceGroupItem):
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
            if isinstance(item, ServiceItem):
                result = group.takeRow(0)
            else:
                result = group._take_node_row(item)
        return result

    def remove_node(self, name):
        removed = False
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == ServiceItem and item == name:
                self.removeRow(i)
                removed = True
                break
            elif type(item) == ServiceGroupItem:
                removed = item.remove_node(name)
                if removed:
                    break
        if removed and self.rowCount() == 0:
            if type(self.parent_item) == ServiceGroupItem:
                self.parent_item._remove_group(self.name)
        return removed

    def update_view(self, updated_srvs, services):
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == ServiceItem:
                if item.service.name in updated_srvs:
                    item.update_view(services[item.service.name])
            elif type(item) == ServiceGroupItem:
                item.update_view(updated_srvs, services)

    def index_from_names(self, services):
        '''
        Returns for given topics the list of QModelIndex in this model.

        :param [str] services: the list of service names
        :return: the list of QModelIndex
        :rtype: [QtCore.QModelIndex]
        '''
        result = []
        for i in range(self.rowCount()):
            item = self.child(i)
            if type(item) == ServiceGroupItem:
                result[len(result):] = item.index_from_names(services)
            elif type(item) == ServiceItem:
                if item.service.name in services:
                    result.append(item.index())
        return result

    def type(self):
        return ServiceGroupItem.ITEM_TYPE

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


class ServiceModel(QStandardItemModel):
    '''
    The model to manage the list with services in ROS network.
    '''
    header = [('Name', 300),
              ('Type', -1)]
    '''@ivar: the list with columns C{[(name, width), ...]}'''

    def __init__(self):
        '''
        Creates a new list model.
        '''
        QStandardItemModel.__init__(self)
        self.setColumnCount(len(ServiceModel.header))
        self.setHorizontalHeaderLabels([label for label, _ in ServiceModel.header])
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the ServiceItem subclass
        topics = ['*/get_loggers', '*/set_logger_level']
        def_list = ['\A' + n.strip().replace('*', '.*') + '\Z' for n in topics]
        self._re_cap_systopics = re.compile('|'.join(def_list), re.I)
        root_items = ServiceGroupItem.create_item_list(rospy.names.SEP, self.invisibleRootItem(), False)
        self.invisibleRootItem().appendRow(root_items)
        self._pyqt_workaround_add(rospy.names.SEP, root_items[0])

    def flags(self, index):
        '''
        :param index: parent of the list
        :type index: QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>
        :return: Flag or the requestet item
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
                if type(item) == ServiceGroupItem:
                    if item == 'SYSTEM' and item.is_group:
                        return item
            items = ServiceGroupItem.create_item_list('SYSTEM', root, True)
            root.appendRow(items)
            self.pyqt_workaround['{SYSTEM}'] = items[0]
            return items[0]
        return None

    def get_root_group(self):
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == ServiceGroupItem:
                if item == rospy.names.SEP:
                    return item
        return None

    def update_model_data(self, services, added_srvs, updated_srvs, removed_srvs):
        '''
        Updates the service list model. New services will be inserted in sorting
        order. Not available services removed from the model.

        :param services: The dictionary with services
        :type services: dict(service name : U{fkie_master_discovery.ServiceInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.ServiceInfo>})
        :param added_srvs: the list of new services in the :service: list
        :type added_srvs: list or set
        :param updated_srvs: the list of updated services in the :service: list
        :type updated_srvs: list or set
        :param removed_srvs: the list of removed services in the :service: list
        :type removed_srvs: list or set
        '''
        # first: remove services
        for rm_service in removed_srvs:
            self._remove_node(rm_service)
        #  second: update the existing items
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            item = root.child(i)
            if type(item) == ServiceGroupItem:
                item.update_view(updated_srvs, services)
        # last: add new services
        for name in added_srvs:
            try:
                service = services[name]
                # first: add to system group
                sys_group = self.get_cap_group(name)
                if sys_group is not None:
                    sys_group.add_node(service)
                else:
                    # second add to the root group
                    root_group = self.get_root_group()
                    if root_group is not None:
                        root_group.add_node(service)
            except Exception:
                import traceback
                print(traceback.format_exc())
                pass
        # remove empty groups
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == ServiceGroupItem:
                item.clearup()

    def index_from_names(self, services):
        '''
        Returns for given services the list of QModelIndex in this model.

        :param [str] services: the list of service names
        :return: the list of QModelIndex
        :rtype: [QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>}]
        '''
        result = []
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == ServiceGroupItem:
                result[len(result):] = item.index_from_names(services)
            elif type(item) == ServiceItem:
                if item.service.name in services:
                    result.append(item.index())
        return result

    def _remove_node(self, name):
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            item = root.child(i)
            if type(item) == ServiceGroupItem:
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
