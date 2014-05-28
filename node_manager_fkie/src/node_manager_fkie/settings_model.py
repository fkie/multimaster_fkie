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

import os
import sys
import shutil

from python_qt_binding import QtCore
from python_qt_binding import QtGui

import node_manager_fkie as nm
from common import is_package, package_name
from packages_thread import PackagesThread
from .detailed_msg_box import WarningMessageBox

class SettingsNameItem(QtGui.QStandardItem):

  ITEM_TYPE = QtGui.QStandardItem.UserType + 80

  def __init__(self, name, tooltip=''):
    QtGui.QStandardItem.__init__(self, name)
    self.name = name
    self.tooltip = tooltip

  def type(self):
    return SettingsNameItem.ITEM_TYPE

  def data(self, role):
    '''
    The view asks us for all sorts of information about our data...
    @param role: the art of the data
    @type role: L{QtCore.Qt.DisplayRole}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    if role == QtCore.Qt.DisplayRole:
      # return the displayed item name
      return self.name
    elif role == QtCore.Qt.ToolTipRole:
      # return the tooltip of the item
      return self.tooltip
    else:
      # We don't care about anything else, so return None
      return QtGui.QStandardItem.data(self, role)


class SettingsValueItem(QtGui.QStandardItem):

  ITEM_TYPE = QtGui.QStandardItem.UserType + 81

  def __init__(self, param_name, default_value, settings=None):
    QtGui.QStandardItem.__init__(self, '%s'%default_value)
    self.name = param_name
    self._param_name = param_name
    self._default_value = default_value
    self._value = default_value
    self._settings = settings

  def type(self):
    return SettingsValueItem.ITEM_TYPE

  def value(self):
    return self._value

  def data(self, role):
    '''
    The view asks us for all sorts of information about our data...
    @param role: the art of the data
    @type role: L{QtCore.Qt.DisplayRole}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    if role == QtCore.Qt.DisplayRole:
      # return the displayed item name
      return '%s'%self._value
#     elif role == QtCore.Qt.DecorationRole:
#       pass
    elif role == QtCore.Qt.EditRole:
      return self._value
    else:
      # We don't care about anything else, so return None
      return QtGui.QStandardItem.data(self, role)

  def setData(self, value, role=QtCore.Qt.EditRole):
    if role == QtCore.Qt.EditRole:
      # rename the file or folder
      self._value = value
      if hasattr(self._settings, self._param_name):
        setattr(self._settings, self._param_name, value)
    return QtGui.QStandardItem.setData(self, value, role)


class SettingsGroupItem(QtGui.QStandardItem):

  ITEM_TYPE = QtGui.QStandardItem.UserType + 82

  def __init__(self, name):
    QtGui.QStandardItem.__init__(self, name)
    self.name = name

  def type(self):
    return SettingsGroupItem.ITEM_TYPE

  def data(self, role):
    '''
    The view asks us for all sorts of information about our data...
    @param index: parent of the list
    @type index: L{QtCore.QModelIndex}
    @param role: the art of the data
    @type role: L{QtCore.Qt.DisplayRole}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    if role == QtCore.Qt.DisplayRole:
      # return the displayed item name
      return self.name
#     elif role == QtCore.Qt.DecorationRole:
#       pass
    else:
      # We don't care about anything else, so return None
      return QtGui.QStandardItem.data(self, role)

  @classmethod
  def getGroupItemList(self, name):
    '''
    Creates the list of the items . This list is used for the 
    visualization of settings group data as a table row.
    @param name: the group name
    @type name: C{str}
    @rtype: C{[L{SettingsGroupItem} and L{PySide.QtGui.QStandardItem}]}
    '''
    items = []
    item = SettingsGroupItem(name)
    items.append(item)
#     item = QtGui.QStandardItem('')
#     items.append(item)
    return items

  @classmethod
  def getSettingsItemList(self, name, param_name, default_value, tooltip='', settings=None):
    '''
    Creates the list of the items . This list is used for the 
    visualization of settings group data as a table row.
    @rtype: C{[L{SettingsGroupItem} and L{PySide.QtGui.QStandardItem}]}
    '''
    items = []
    item = SettingsNameItem(name, tooltip)
    items.append(item)
    item = SettingsValueItem(param_name, default_value, settings)
    items.append(item)
    return items


class SettingsModel(QtGui.QStandardItemModel):
  '''
  The model to manage the settings.
  '''
  header = [('Parameter', 160), ('Value', -1)]
  '''@ivar: the list with columns C{[(name, width), ...]}'''

  def __init__(self):
    '''
    Creates a new list model.
    '''
    QtGui.QStandardItemModel.__init__(self)
    self.setColumnCount(len(SettingsModel.header))
    self.setHorizontalHeaderLabels([label for label, width in SettingsModel.header])
    self.pyqt_workaround = dict() # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Overloaded methods                    %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def flags(self, index):
    '''
    @param index: parent of the list
    @type index: L{PySide.QtCore.QModelIndex}
    @return: Flag or the requested item
    @rtype: L{PySide.QtCore.Qt.ItemFlag}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    if not index.isValid():
      return QtCore.Qt.NoItemFlags
    try:
      item = self.itemFromIndex(index)
      result = QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled
      if item.type() in [SettingsValueItem.ITEM_TYPE]:
        result = result | QtCore.Qt.ItemIsEditable
      return result
    except:
      import traceback
      print traceback.format_exc()
      return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              External usage                        %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def init_settings(self, settings):
    '''
    Updates the model data.
    @param settings: a dictionary with settings name and values.
    @type settings: C{dict(str:L{(.. parameter of SettingsGroupItem.getSettingsItemList())}, ...)}
    '''
    # remove all current items
    root = self.invisibleRootItem()
    while root.rowCount():
      root.removeRow(0)
    self.pyqt_workaround.clear()
    # add new items
    try:
      for name, value in settings.items():
        self._add_item(root, name, value)
    except:
      import traceback
      print traceback.format_exc()

  def _add_item(self, root, name, value):
    if isinstance(value, dict):
      new_item_row = SettingsGroupItem.getGroupItemList(name)
      root.appendRow(new_item_row)
      self.pyqt_workaround['group_%s'%name] = new_item_row[0]
      for name, value in value.items():
        self._add_item(new_item_row[0], name, value)
    else:
      new_item_row = SettingsGroupItem.getSettingsItemList(value[1], name, value[0], value[2], value[3])
      root.appendRow(new_item_row)
      self.pyqt_workaround[name] = new_item_row[0]
