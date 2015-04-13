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

from python_qt_binding import QtCore
from python_qt_binding import QtGui

import threading

from urlparse import urlparse
from socket import getaddrinfo, AF_INET, AF_INET6

import node_manager_fkie as nm

class MasterSyncItem(QtGui.QStandardItem):
  ITEM_TYPE = QtGui.QStandardItem.UserType + 35

  NOT_SYNC = 0
  START_SYNC = 1
  SYNC = 2

  ICON_PREFIX = 'irondevil'
#  ICON_PREFIX = 'crystal_clear'

  def __init__(self, master, parent=None):
    self.name = master.name
    QtGui.QStandardItem.__init__(self, self.name)
    self.parent_item = None
    self._master = master
    self._syncronized = MasterSyncItem.NOT_SYNC
    self.ICONS = {MasterSyncItem.SYNC : QtGui.QIcon(":/icons/%s_sync.png"%self.ICON_PREFIX),
                  MasterSyncItem.NOT_SYNC : QtGui.QIcon(":/icons/%s_not_sync.png"%self.ICON_PREFIX),
                  MasterSyncItem.START_SYNC: QtGui.QIcon(":/icons/%s_start_sync.png"%self.ICON_PREFIX) }
    self.setIcon(self.ICONS[MasterSyncItem.NOT_SYNC])

  @property
  def master(self):
    return self._master
  
  @property
  def synchronized(self):
    return self._syncronized

  @synchronized.setter
  def synchronized(self, value):
    if self._syncronized != value:
      self._syncronized = value
      if value in self.ICONS:
        self.setIcon(self.ICONS[value])

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


class MasterItem(QtGui.QStandardItem):
  '''
  The master item stored in the master model. This class stores the master as
  master_discovery_fkie.ROSMaster.
  '''
  
  ITEM_TYPE = QtGui.QStandardItem.UserType + 34

  def __init__(self, master, local=False, quality=None, parent=None):
    self.name = ''.join([master.name, ' (localhost)']) if local else master.name
    QtGui.QStandardItem.__init__(self, self.name)
    self.parent_item = None
    self._master = master
    self.__quality = quality
    self.descr = ''
    self.ICONS = {'green' : QtGui.QIcon(":/icons/stock_connect_green.png"),
                  'yellow': QtGui.QIcon(":/icons/stock_connect_yellow.png"),
                  'red'   : QtGui.QIcon(":/icons/stock_connect_red.png"),
                  'grey'  : QtGui.QIcon(":/icons/stock_connect.png"),
                  'disconnected' : QtGui.QIcon(":/icons/stock_disconnect.png"),
                  'warning' : QtGui.QIcon(':/icons/crystal_clear_warning.png') }
    self.master_ip = None
    self._master_errors = []
    self._threaded_get_ip()
    self.updateNameView(master, quality, self)

  def _threaded_get_ip(self):
    thread = threading.Thread(target=self.__get_ip)
    thread.daemon = True
    thread.start()

  def __get_ip(self):
    try:
      # get the IP of the master uri
      o = urlparse(self.master.uri)
      result = getaddrinfo(o.hostname, None)
      ips = []
      for r in result:
        if r[0] == AF_INET6:
          (family, socktype, proto, canonname, (ip, port, flow, scope)) = r
        else:
          (family, socktype, proto, canonname, (ip, port)) = r
        if self.master_ip is None and ip:
          self.master_ip = ''
        if ip and not ip in ips:
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

  def updateMasterView(self, parent):
    '''
    This method is called after the master state is changed to update the 
    representation of the master. The name will not be changed, but all other 
    data.
    @param parent: Item which contains this master item. This is needed to update 
    other columns of this master.
    @type parent: L{PySide.QtGui.QStandardItem}
    '''
    if not parent is None:
      #update the name decoration
      child = parent.child(self.row(), MasterModel.COL_NAME)
      if not child is None:
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
      if not quality is None and quality != -1.:
        tooltip = ''.join([tooltip, '<dt>', 'Quality: ', str(quality),' %', '</dt>'])
      else:
        tooltip = ''.join([tooltip, '<dt>', 'Quality: not available</dt>'])
#      if item.checkState() == QtCore.Qt.Checked:
#        tooltip = ''.join([tooltip, '<dt>', 'synchronized', '</dt>'])
    else:
      tooltip = ''.join([tooltip, '<dt>', 'offline', '</dt>'])
    tooltip = ''.join([tooltip, '</dl>'])
    if item.descr:
#      tooltip = ''.join([tooltip, '<b><u>Description:</u></b>'])
      tooltip = ''.join([tooltip, item.descr])
    # update the icon
    if master.online:
      if self._master_errors or self.master_ip is None:
        item.setIcon(self.ICONS['warning'])
        if self.master_ip is None:
          tooltip = ''.join([tooltip, '<h4>', '<font color="#CC0000">Host not reachable by name!!! The ROS topics may not by connected!!!</font>', '</h4>'])
        if self._master_errors:
          tooltip = ''.join([tooltip, '<h4>Errors reported by master_discovery:</h4>'])
          for err in self._master_errors:
            tooltip = ''.join([tooltip, '<dt><font color="#CC0000">%s</font></dt>'%err])
      elif not quality is None and quality != -1.:
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

  @classmethod
  def getItemList(self, master, local):
    '''
    Creates the list of the items from master. This list is used for the 
    visualization of master data as a table row.
    @param master the master data
    @type master master_discovery_fkie.ROSMaster
    @param local: whether the master is local or not
    @type local: bool
    @return: the list for the representation as a row
    @rtype: C{[L{MasterItem} or L{PySide.QtGui.QStandardItem}, ...]}
    '''
    items = []
    item = MasterSyncItem(master)
    items.append(item)
    item = MasterItem(master, local)
    items.append(item)
    return items

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



class MasterModel(QtGui.QStandardItemModel):
  '''
  The model to manage the list with masters in ROS network.
  '''
  header = [('Sync', 22), ('Name', -1)]
  '''@ivar: the list with columns C{[(name, width), ...]}'''
  COL_SYNC = 0
  COL_NAME = 1

  def __init__(self, local_masteruri=None):
    '''
    Creates a new list model.
    '''
    QtGui.QStandardItemModel.__init__(self)
    self.setColumnCount(len(MasterModel.header))
    self._masteruri = local_masteruri
    self.pyqt_workaround = dict() # workaround for using with PyQt: store the python object to keep the defined attributes in the MasterItem subclass

  def flags(self, index):
    '''
    @param index: parent of the list
    @type index: L{PySide.QtCore.QModelIndex}
    @return: Flag or the requestet item
    @rtype: L{PySide.QtCore.Qt.ItemFlag}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    if not index.isValid():
      return QtCore.Qt.NoItemFlags
#    item = self.itemFromIndex(index)
#    if item and item.master.online:
    return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled
#    return QtCore.Qt.NoItemFlags

  def updateMaster(self, master):
    '''
    Updates the information of the ros master. If the ROS master not exists, it 
    will be added.
    
    @param master: the ROS master to update
    @type master: L{master_discovery_fkie.msg.ROSMaster}
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
    for i in range(root.rowCount()):
      masterItem = root.child(i, self.COL_NAME)
      if (masterItem == master.name):
        # update item
        masterItem.master = master
        masterItem.updateMasterView(root)
        doAddItem = False
        break
      elif (masterItem > master.name):
        mitem = MasterItem.getItemList(master, (nm.is_local(nm.nameres().getHostname(master.uri))))
        self.pyqt_workaround[master.name] = mitem  # workaround for using with PyQt: store the python object to keep the defined attributes in the MasterItem subclass
        root.insertRow(i, mitem)
        mitem[self.COL_NAME].parent_item = root
        doAddItem = False
        break
    if doAddItem:
      mitem = MasterItem.getItemList(master, (nm.is_local(nm.nameres().getHostname(master.uri))))
      self.pyqt_workaround[master.name] = mitem  # workaround for using with PyQt: store the python object to keep the defined attributes in the MasterItem subclass
      root.appendRow(mitem)
      mitem[self.COL_NAME].parent_item = root

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
        masterItem.synchronized = MasterSyncItem.SYNC if state else MasterSyncItem.NOT_SYNC
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
      if masterItem.master.name in master:
        masterItem.updateMasterErrors(errors)
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
