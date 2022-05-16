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


import sys
from python_qt_binding.QtCore import QObject, QRect, Qt, Signal, QEvent
from python_qt_binding.QtGui import QIcon, QImage, QStandardItem, QStandardItemModel
try:
    from python_qt_binding.QtGui import QItemDelegate, QPushButton, QStyle
except Exception:
    from python_qt_binding.QtWidgets import QItemDelegate, QPushButton, QStyle, QStyledItemDelegate
from socket import getaddrinfo, AF_INET6
import threading

from fkie_master_discovery.master_discovery import DiscoveredMaster
from fkie_master_discovery.common import get_hostname
from fkie_node_manager_daemon.common import isstring
import fkie_node_manager as nm


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
        self.ICONS = {MasterSyncButtonHelper.SYNC: nm.settings().icon("%s_sync.png" % self.ICON_PREFIX),
                      MasterSyncButtonHelper.NOT_SYNC: nm.settings().icon("%s_not_sync.png" % self.ICON_PREFIX),
                      MasterSyncButtonHelper.SWITCHED: nm.settings().icon("%s_start_sync.png" % self.ICON_PREFIX)}
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
        if isstring(item):
            return self.master.name.lower() == item.lower()
        elif not (item is None):
            return self.master.name.lower() == item.master.name.lower()
        return False

    def __gt__(self, item):
        if isstring(item):
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
        self.model().setData(self.index(), value)

    def __eq__(self, item):
        return self.button == item

    def __gt__(self, item):
        return self.button > item


class MasterItem(QStandardItem):
    '''
    The master item stored in the master model. This class stores the master as
    fkie_master_discovery.ROSMaster.
    '''

    ITEM_TYPE = QStandardItem.UserType + 34

    def __init__(self, master, local=False, quality=None, parent=None):
        self.name = ''.join([master.name, ' (localhost)']) if local else master.name
        QStandardItem.__init__(self, '')  # self.name)
        self.parent_item = None
        self._master = master
        self.local = local
        self.__quality = quality
        self.descr = ''
        self.ICONS = {'green': nm.settings().icon('stock_connect_green.png'),
                      'yellow': nm.settings().icon('stock_connect_yellow.png'),
                      'red': nm.settings().icon('stock_connect_red.png'),
                      'grey': nm.settings().icon('stock_connect.png'),
                      'disconnected': nm.settings().icon('stock_disconnect.png'),
                      'warning': nm.settings().icon('crystal_clear_warning.png'),
                      'clock_warn': nm.settings().icon('crystal_clear_xclock_fail.png')}
        self.master_ip = None
        self._master_errors = []
        self._diagnostics = []
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
        except Exception:
            import traceback
            print(traceback.format_exc(1))

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

    @property
    def diagnostics(self):
        return list(self._diagnostics)

    @property
    def master_errors(self):
        return list(self._master_errors)

    def updateMasterErrors(self, error_list):
        self._master_errors = error_list
        self.updateNameView(self.master, self.quality, self)

    def add_master_error(self, msg):
        if msg not in self._master_errors:
            self._master_errors.append(msg)
            self.updateNameView(self.master, self.quality, self)

    def update_master_diagnostics(self, diagnostics):
        del self._diagnostics[:]
        for diagnostic in diagnostics.status:
            if diagnostic.level > 0 and diagnostic.hardware_id == self._master.name:
                self._diagnostics.append(diagnostic)
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
        @type master: fkie_master_discovery.TopicInfo
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
                tooltip = ''.join([tooltip, '<dt>', 'Quality: not available, start <b>master_discovery</b> with <b>heartbeat_hz</b> parameter >= %.02f</dt>' % DiscoveredMaster.MIN_HZ_FOR_QUALILTY])
        else:
            tooltip = ''.join([tooltip, '<dt>', 'offline', '</dt>'])
        tooltip = ''.join([tooltip, '</dl>'])
        if item.descr:
            tooltip = ''.join([tooltip, item.descr])
        # update the icon
        if master.online:
            timediff = abs(self._timediff) > nm.settings().max_timediff
            if self._master_errors or self._diagnostics or self.master_ip is None or timediff:
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
                for diag in self._diagnostics:
                    tooltip = ''.join([tooltip, '<dt><font color="#CC0000">%s</font></dt>' % diag.message])
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

    def update_description(self, descr):
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
        if isstring(item):
            return self.master.name.lower() == item.lower()
        elif not (item is None):
            return self.master.name.lower() == item.master.name.lower()
        return False

    def __gt__(self, item):
        if isstring(item):
            local = False
            try:
                local = nm.is_local(get_hostname(nm.nameres().masteruri(item)))
            except Exception:
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
        @type master: U{fkie_master_discovery.msg.ROSMaster<http://docs.ros.org/api/fkie_multimaster_msgs/html/msg/ROSMaster.html>}
        '''
        # remove master, if his name was changed but not the ROS master URI
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i)
            if masterItem.master.uri == master.uri and masterItem.master.name != master.name:
                root.removeRow(i)
                try:
                    del self.pyqt_workaround[masterItem.master.name]
                except Exception:
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
        @type master: fkie_master_discovery.ROSMaster
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
                except Exception:
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

    def add_master_error(self, master, msg):
        '''
        Add error to the error list.

        :param str master: the ROS master to update
        :param str msg: error message
        '''
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i, self.COL_NAME)
            if masterItem.master.name == master:
                masterItem.add_master_error(msg)
                break

    def update_master_diagnostic(self, master_name, diagnostics):
        root = self.invisibleRootItem()
        for i in reversed(range(root.rowCount())):
            masterItem = root.child(i, self.COL_NAME)
            if masterItem.master.name == master_name:
                masterItem.update_master_diagnostics(diagnostics)
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

    def update_description(self, master, descr):
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
                masterItem.update_description(descr)

    def on_sync_clicked(self, checked, masteruri):
        if checked:
            self.sync_start.emit(masteruri)
        else:
            self.sync_stop.emit(masteruri)


class MasterIconsDelegate(QItemDelegate):

    def __init__(self, parent=None, *args):
        QItemDelegate.__init__(self, parent, *args)
        self._idx_icon = 1
        self._hspacing = 2
        self._vspacing = 4
        self._icon_size = 0
        self._enabled = True
        self.IMAGES = {}

    def _scale_icons(self, icon_size):
        self._icon_size = icon_size
        params = (self._icon_size, self._icon_size, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        self.IMAGES = {'green': nm.settings().image('stock_connect_green.png').scaled(*params),
                       'yellow': nm.settings().image('stock_connect_yellow.png').scaled(*params),
                       'red': nm.settings().image('stock_connect_red.png').scaled(*params),
                       'grey': nm.settings().image('stock_connect.png').scaled(*params),
                       'disconnected': nm.settings().image('stock_disconnect.png').scaled(*params),
                       'warning': nm.settings().image('crystal_clear_warning.png').scaled(*params),
                       'clock_warn': nm.settings().image('crystal_clear_xclock_fail.png').scaled(*params),
                       'cpu_warn': nm.settings().image('hight_load.png').scaled(*params),
                       'cpu_temp_warn': nm.settings().image('temperatur_warn.png').scaled(*params),
                       'hdd_warn': nm.settings().image('crystal_clear_hdd_warn.png').scaled(*params),
                       'net_warn': nm.settings().image('sekkyumu_net_warn.png').scaled(*params),
                       'mem_warn': nm.settings().image('mem_warn.png').scaled(*params)
                       }

    def set_enabled(self, value):
        self._enabled = value

    def paint(self, painter, option, index):
        # update the icon size and resize images if needed
        if option.rect.height() - self._vspacing * 2 != self._icon_size:
            self._icon_size = option.rect.height() - self._vspacing * 2
            self._scale_icons(self._icon_size)
        painter.save()
        self._idx_icon = 1
        item = index.model().itemFromIndex(index)
        if option.state & QStyle.State_Selected:
            painter.fillRect(option.rect, option.palette.highlight())

        if isinstance(item, MasterItem):
            tooltip = '<html><body>'
            tooltip = '%s\n<h4>%s</h4>' % (tooltip, item.master.uri)
            tooltip = '%s\n<dt>IP: %s</dt>' % (tooltip, str(item.master_ip))
            if item.master.online:
                if item.quality is not None and item.quality != -1.:
                    tooltip = '%s\n<dt>Quality: %.2f </dt>' % (tooltip, item.quality)
                else:
                    tooltip = '%s\n<dt>Quality: not available, start <b>master_discovery</b> with <b>heartbeat_hz</b> parameter >= %.02f</dt>' % (tooltip, DiscoveredMaster.MIN_HZ_FOR_QUALILTY)
            else:
                tooltip = '%s\n<dt>offline</dt>' % (tooltip)
            # update warnings
            if item.master.online:
                master_errors = item.master_errors
                if master_errors or item.master_ip is None:
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['warning'])
                    if item.master_ip is None:
                        tooltip = '%s\n<h4><font color="#CC0000">Host not reachable by name! The ROS topics may not by connected!</font></h4>' % (tooltip)
                    if master_errors:
                        tooltip = '%s\n<h4>Errors reported by master_discovery:</h4>' % (tooltip)
                        for err in master_errors:
                            tooltip = '%s\n<dt><font color="#CC0000">%s</font></dt>' % (tooltip, err)
                elif self._enabled:
                    rect = self.calcDecorationRect(option.rect)
                    if item.quality is not None and item.quality != -1.:
                        if item.quality > 30:
                            painter.drawImage(rect, self.IMAGES['green'])
                        elif item.quality > 5:
                            painter.drawImage(rect, self.IMAGES['yellow'])
                        else:
                            painter.drawImage(rect, self.IMAGES['red'])
                    else:
                        painter.drawImage(rect, self.IMAGES['grey'])
                # check for time difference
                timediff = abs(item._timediff) > nm.settings().max_timediff
                if timediff:
                    tooltip = '%s\n<h4><font color="#CC0000">Time difference to the host is about %.3f seconds!</font></h4>' % (tooltip, item._timediff)
                    rect = self.calcDecorationRect(option.rect)
                    painter.drawImage(rect, self.IMAGES['clock_warn'])
            else:
                rect = self.calcDecorationRect(option.rect)
                painter.drawImage(rect, self.IMAGES['disconnected'])
            # update diagnostic warnings
            for diag in item.diagnostics:
                if diag.level > 0:
                    tooltip = '%s\n<dt><font color="#CC0000">%s</font></dt>' % (tooltip, diag.message.replace('>', '&gt;').replace('<', '&lt;'))
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
            # update description from robot description parameter
            if item.descr:
                tooltip = '%s\n%s' % (tooltip, item.descr)

            # paint the name of the host
            tooltip = '%s\n</body></html>' % (tooltip)
            item.setToolTip(tooltip)
            rect = self.calcDecorationRect(option.rect, image=False)
            painter.drawText(rect, Qt.AlignVCenter, item.name)
        painter.restore()

    def calcDecorationRect(self, main_rect, image=True):
        rect = QRect()
        rect.setX(main_rect.x() + self._idx_icon + self._hspacing)
        rect.setY(main_rect.y() + self._vspacing)
        rect.setWidth(self._icon_size if image else main_rect.width() - self._idx_icon)
        rect.setHeight(self._icon_size)
        self._idx_icon += self._icon_size + self._hspacing
        return rect


class MasterButtonDelegate(QStyledItemDelegate):

    def editorEvent(self, event, model, option, index):

        if event.type() == QEvent.MouseButtonRelease:
            item = index.model().itemFromIndex(index)
            item.button.widget.click()
            return True
        return QStyledItemDelegate.editorEvent(self, event, model, option, index)


    def paint(self, painter, option, index):
        item = index.model().itemFromIndex(index)
        item.button.widget.setGeometry(option.rect)
        painter.save()
        painter.translate(option.rect.topLeft())
        item.button.widget.render(painter)
        painter.restore()
