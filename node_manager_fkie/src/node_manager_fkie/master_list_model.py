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

from python_qt_binding.QtCore import QObject, Qt, Signal
from python_qt_binding.QtGui import QIcon, QStandardItem, QStandardItemModel
try:
    from python_qt_binding.QtGui import QPushButton
except:
    from python_qt_binding.QtWidgets import QPushButton
from socket import getaddrinfo, AF_INET6
import threading

from master_discovery_fkie.common import get_hostname
import node_manager_fkie as nm


class MasterSyncButtonHelper(QObject):
    '''
    This is helper class to which contains a button and can emit signals. The
    MasterSyncItem can not emit signals, but is used in QStandardModel.
    '''
    clicked = Signal(bool, str)

    NOT_SYNC = 0
    SWITCHED = 1
    SYNC = 2

    ICON_PREFIX = 'irondevil'
#  ICON_PREFIX = 'crystal_clear'

    def __init__(self, master):
        QObject.__init__(self)
        self.name = master.name
        self._master = master
        self._syncronized = MasterSyncButtonHelper.NOT_SYNC
        self.ICONS = {MasterSyncButtonHelper.SYNC: QIcon(":/icons/%s_sync.png" % self.ICON_PREFIX),
                      MasterSyncButtonHelper.NOT_SYNC: QIcon(":/icons/%s_not_sync.png" % self.ICON_PREFIX),
                      MasterSyncButtonHelper.SWITCHED: QIcon(":/icons/%s_start_sync.png" % self.ICON_PREFIX)}
        self.widget = QPushButton()
#    self.widget.setFlat(True)
        self.widget.setIcon(self.ICONS[MasterSyncButtonHelper.NOT_SYNC])
        self.widget.setMaximumSize(48, 48)
        self.widget.setCheckable(True)
        self.widget.clicked.connect(self.on_sync_clicked)

    def on_sync_clicked(self, checked):
        self.set_sync_state(MasterSyncButtonHelper.SWITCHED)
        self.clicked.emit(checked, self._master.uri)

    def master(self):
        return self._master

    def get_sync_state(self):
        return self._syncronized

    def set_sync_state(self, value):
        if self._syncronized != value:
            self._syncronized = value
            if value in self.ICONS:
                self.widget.setIcon(self.ICONS[value])
                self.widget.setChecked(value == MasterSyncButtonHelper.SYNC)

    def __eq__(self, item):
        if isinstance(item, str) or isinstance(item, unicode):
            return self.master.name.lower() == item.lower()
        elif not (item is None):
            return self.master.name.lower() == item.master.name.lower()
        return False

    def __gt__(self, item):
        if isinstance(item, str) or isinstance(item, unicode):
            return self.master.name.lower() > item.lower()
        elif not (item is None):
            return self.master.name.lower() > item.master.name.lower()
        return False


class MasterSyncItem(QStandardItem):
    '''
    This object is needed to insert into the QStandardModel.
    '''
    ITEM_TYPE = QStandardItem.UserType + 35

    def __init__(self, master):
        QStandardItem.__init__(self)
        self.name = master.name
        self.button = MasterSyncButtonHelper(master)
        self.parent_item = None

    @property
    def master(self):
        return self.button.master()

    @property
    def synchronized(self):
        return self.button.get_sync_state()

    @synchronized.setter
    def synchronized(self, value):
        self.button.set_sync_state(value)

    def __eq__(self, item):
        return self.button == item

    def __gt__(self, item):
        return self.button > item


class MasterItem(QStandardItem):
    '''
    The master item stored in the master model. This class stores the master as
    master_discovery_fkie.ROSMaster.
    '''

    ITEM_TYPE = QStandardItem.UserType + 34

    def __init__(self, master, local=False, quality=None, parent=None):
        self.name = ''.join([master.name, ' (localhost)']) if local else master.name
        QStandardItem.__init__(self, self.name)
        self.parent_item = None
        self._master = master
        self.local = local
        self.__quality = quality
        self.descr = ''
        self.ICONS = {'green': QIcon(":/icons/stock_connect_green.png"),
                      'yellow': QIcon(":/icons/stock_connect_yellow.png"),
                      'red': QIcon(":/icons/stock_connect_red.png"),
                      'grey': QIcon(":/icons/stock_connect.png"),
                      'disconnected': QIcon(":/icons/stock_disconnect.png"),
                      'warning': QIcon(':/icons/crystal_clear_warning.png'),
                      'clock_warn': QIcon(':/icons/crystal_clear_xclock_fail.png')}
        self.master_ip = None
        self._master_errors = []
        self._timediff = 0
        self._threaded_get_ip()
        self.updateNameView(master, quality, self)

    def _threaded_get_ip(self):
        thread = threading.Thread(target=self.__get_ip)
        thread.daemon = True
        thread.start()

    def __get_ip(self):
        try:
            # get the IP of the master uri
            result = getaddrinfo(get_hostname(self.master.uri), None)
            ips = []
            for r in result:
                if r[0] == AF_INET6:
                    (_family, _socktype, _proto, _canonname, (ip, _port, _flow, _scope)) = r
                else:
                    (_family, _socktype, _proto, _canonname, (ip, _port)) = r
                if self.master_ip is None and ip:
                    self.master_ip = ''
                if ip and ip not in ips:
                    self.master_ip = ' '.join([self.master_ip, ip])
                    ips.append(ip)
#      self.updateNameView(self.master, self.quality, self)
        except:
            import traceback
            print traceback.format_exc(1)

    @property
    def master(self):
        return self._master

    @master.setter
    def master(self, value):
        self._master = value

    @property
    def quality(self):
        return self.__quality

    @quality.setter
    def quality(self, value):
        if self.__quality != value:
            self.__quality = value
            self.updateMasterView(self.parent_item)

    def updateMasterErrors(self, error_list):
        self._master_errors = error_list
        self.updateNameView(self.master, self.quality, self)

    def updateTimeDiff(self, timediff):
        self._timediff = timediff
        self.updateNameView(self.master, self.quality, self)

    def updateMasterView(self, parent):
        '''
        This method is called after the master state is changed to update the
        representation of the master. The name will not be changed, but all other
        data.
        @param parent: Item which contains this master item. This is needed to update
        other columns of this master.
        @type parent: U{QtGui.QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}
        '''
        if parent is not None:
            # update the name decoration
            child = parent.child(self.row(), MasterModel.COL_NAME)
            if child is not None:
                self.updateNameView(self.master, self.quality, child)

    def updateNameView(self, master, quality, item):
        '''
        Updates the representation of the column contains the name state.
        @param master: the topic data
        @type master: master_discovery_fkie.TopicInfo
        @param item: corresponding item in the model
        @type item: L{TopicItem}
        '''
        tooltip = ''.join(['<html><body>'])
        tooltip = ''.join([tooltip, '<h4>', master.uri, '</h4>'])
        tooltip = ''.join([tooltip, '<dl>'])
        tooltip = ''.join([tooltip, '<dt>', 'IP: ', str(self.master_ip), '</dt>'])
        if master.online:
            if quality is not None and quality != -1.:
                tooltip = ''.join([tooltip, '<dt>', 'Quality: ', str(quality), ' %', '</dt>'])
            else:
                tooltip = ''.join([tooltip, '<dt>', 'Quality: not available</dt>'])
        else:
            tooltip = ''.join([tooltip, '<dt>', 'offline', '</dt>'])
        tooltip = ''.join([tooltip, '</dl>'])
        if item.descr:
            tooltip = ''.join([tooltip, item.descr])
        # update the icon
        if master.online:
            timediff = abs(self._timediff) > nm.settings().max_timediff
            if self._master_errors or self.master_ip is None or timediff:
                item.setIcon(self.ICONS['warning'])
                if timediff:
                    tooltip = ''.join([tooltip, '<h4>', '<font color="#CC0000">Time difference to the host is about %.3f seconds!</font>' % self._timediff, '</h4>'])
                    item.setIcon(self.ICONS['clock_warn'])
                if self.master_ip is None:
                    tooltip = ''.join([tooltip, '<h4>', '<font color="#CC0000">Host not reachable by name!!! The ROS topics may not by connected!!!</font>', '</h4>'])
                if self._master_errors:
                    tooltip = ''.join([tooltip, '<h4>Errors reported by master_discovery:</h4>'])
                    for err in self._master_errors:
                        tooltip = ''.join([tooltip, '<dt><font color="#CC0000">%s</font></dt>' % err])
            elif quality is not None and quality != -1.:
                if quality > 30:
                    item.setIcon(self.ICONS['green'])
                elif quality > 5:
                    item.setIcon(self.ICONS['yellow'])
                else:
                    item.setIcon(self.ICONS['red'])
            else:
                item.setIcon(self.ICONS['grey'])
        else:
            item.setIcon(self.ICONS['disconnected'])

        tooltip = ''.join([tooltip, '</body></html>'])
        item.setToolTip(tooltip)

    def updateDescription(self, descr):
        self.descr = descr
        self.updateNameView(self.master, self.quality, self)

    @classmethod
    def toHTML(cls, text):
        '''
        @param text: the text
        @type text: C{str}
        @return: the HTML representation of the name of the text
        @rtype: C{str}
        '''
        ns, sep, name = text.rpartition('/')
        result = ''
        if sep:
            result = ''.join(['<html><body>', '<span style="color:gray;">', str(ns), sep, '</span><b>', name, '</b></body></html>'])
        else:
            result = name
        return result

    def type(self):
        return MasterItem.ITEM_TYPE

    def __eq__(self, item):
        if isinstance(item, str) or isinstance(item, unicode):
            return self.master.name.lower() == item.lower()
        elif not (item is None):
            return self.master.name.lower() == item.master.name.lower()
        return False

    def __gt__(self, item):
        if isinstance(item, str) or isinstance(item, unicode):
            local = False
            try:
                local = nm.is_local(item)
            except:
                pass
            if self.local and not local:  # local hosts are at the top
                return False
            return self.master.name.lower() > item.lower()
        elif not (item is None):
            if self.local and not item.local:  # local hosts are at the top
                return False
            return self.master.name.lower() > item.master.name.lower()
        return False


class MasterModel(QStandardItemModel):
    '''
    The model to manage the list with masters in ROS network.
    '''
    sync_start = Signal(str)
    sync_stop = Signal(str)

    header = [('Sync', 28), ('Name', -1)]
    '''@ivar: the list with columns C{[(name, width), ...]}'''
    COL_SYNC = 0
    COL_NAME = 1
    COL_SYNCBTN = 2

    def __init__(self, local_masteruri=None):
        '''
        Creates a new list model.
        '''
        QStandardItemModel.__init__(self)
        self.setColumnCount(len(MasterModel.header))
        self._masteruri = local_masteruri
        self.parent_view = None
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the MasterItem subclass

    def flags(self, index):
        '''
        @param index: parent of the list
        @type index: U{QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>}
        @return: Flag or the requestet item
        @rtype: U{QtCore.Qt.ItemFlag<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if not index.isValid():
            return Qt.NoItemFlags
#    item = self.itemFromIndex(index)
#    if item and item.master.online:
        return Qt.ItemIsSelectable | Qt.ItemIsEnabled
#    return Qt.NoItemFlags

    def updateMaster(self, master):
        '''
        Updates the information of the ros master. If the ROS master not exists, it
        will be added.

        @param master: the ROS master to update
        @type master: U{master_discovery_fkie.msg.ROSMaster<http://docs.ros.org/api/multimaster_msgs_fkie/html/msg/ROSMaster.html>}
        '''
        # remove master, if his name was changed but not the ROS master URI
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i)
            if masterItem.master.uri == master.uri and masterItem.master.name != master.name:
                root.removeRow(i)
                try:
                    del self.pyqt_workaround[masterItem.master.name]
                except:
                    pass
                break

        # update or add a the item
        root = self.invisibleRootItem()
        doAddItem = True
        is_local = nm.is_local(get_hostname(master.uri))
        for index in range(root.rowCount()):
            masterItem = root.child(index, self.COL_NAME)
            if (masterItem == master.name):
                # update item
                masterItem.master = master
                masterItem.updateMasterView(root)
                doAddItem = False
                break
            elif (masterItem > master.name):
                self.addRow(master, is_local, root, index)
                doAddItem = False
                break
        if doAddItem:
            self.addRow(master, is_local, root, -1)

    def addRow(self, master, local, root, index):
        '''
        Creates the list of the items from master. This list is used for the
        visualization of master data as a table row.
        @param master: the master data
        @type master: master_discovery_fkie.ROSMaster
        @param local: whether the master is local or not
        @type local: bool
        @return: the list for the representation as a row
        @rtype: C{[L{MasterItem} or U{QtGui.QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}, ...]}
        '''
        items = []
        sync_item = MasterSyncItem(master)
        items.append(sync_item)
        name_item = MasterItem(master, local)
        items.append(name_item)
        name_item.parent_item = root
        self.pyqt_workaround[master.name] = items  # workaround for using with PyQt: store the python object to keep the defined attributes in the MasterItem subclass
        # add the items to the data model
        if index > -1:
            root.insertRow(index, items)
        else:
            root.appendRow(items)
        # add the sync botton and connect the signals
        if self.parent_view is not None:
            newindex = index if index > -1 else root.rowCount() - 1
            self.parent_view.setIndexWidget(self.index(newindex, self.COL_SYNC), sync_item.button.widget)
            sync_item.button.clicked.connect(self.on_sync_clicked)
        return items

    def updateMasterStat(self, master, quality):
        '''
        Updates the information of the ros master.

        @param master: the ROS master to update
        @type master: C{str}
        @param quality: the quality of the connection to master
        @type quality: C{float}
        '''
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i, self.COL_NAME)
            if masterItem.master.name in master:
                masterItem.quality = quality
                break

    def setChecked(self, master, state):
        '''
        Set the master to checked state

        @param master: the ROS master to update
        @type master: C{str}
        @param state: new state
        @type state: C{bool}
        '''
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i, self.COL_SYNC)
            if masterItem.master.name == master:
                masterItem.synchronized = MasterSyncButtonHelper.SYNC if state else MasterSyncButtonHelper.NOT_SYNC
                break

    def removeMaster(self, master):
        '''
        Remove the master with given name.

        @param master: the ROS master to add
        @type master: C{str}
        '''
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i, self.COL_NAME)
            if masterItem.master.name == master:
                root.removeRow(i)
                try:
                    del self.pyqt_workaround_sync[masterItem.master.name]
                    del self.pyqt_workaround_info[masterItem.master.name]
                except:
                    pass
                break

    def updateMasterErrors(self, master, errors):
        '''
        Updates the errors reported by master_discovery.

        @param master: the ROS master to update
        @type master: C{str}
        @param errors: the list with errors
        @type errors: C{[str]}
        '''
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i, self.COL_NAME)
            if masterItem.master.name == master:
                masterItem.updateMasterErrors(errors)
                break

    def updateTimeDiff(self, master, timediff):
        '''
        Updates the time difference reported by master_discovery.

        @param master: the ROS master to update
        @type master: C{str}
        @param timediff: the time difference to the host
        @type timediff: float
        '''
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i, self.COL_NAME)
            if masterItem.master.name == master:
                masterItem.updateTimeDiff(timediff)
                break

    def updateDescription(self, master, descr):
        '''
        Updates the description of the master with given name.

        @param master: the ROS master to add
        @type master: C{str}
        @param descr: the description of the master coded as HTML
        @type descr: C{str}
        '''
        root = self.invisibleRootItem()
        for i in range(root.rowCount()):
            masterItem = root.child(i, self.COL_NAME)
            if masterItem and masterItem.master.name == master:
                masterItem.updateDescription(descr)

    def on_sync_clicked(self, checked, masteruri):
        if checked:
            self.sync_start.emit(masteruri)
        else:
            self.sync_stop.emit(masteruri)
