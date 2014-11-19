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
from xmlrpclib import Binary


class ParameterValueItem(QtGui.QStandardItem):
  '''
  The parameter item is stored in the parameter model. This class stores the name 
  and value of a parameter of ROS parameter server and shows the value.
  '''

  ITEM_TYPE = QtGui.QStandardItem.UserType + 39
  NAME_ROLE = QtCore.Qt.UserRole + 1
  VALUE_ROLE = QtCore.Qt.UserRole + 2
  TYPE_ROLE = QtCore.Qt.UserRole + 3

  def __init__(self, name, value, parent=None):
    '''
    Initialize the item object.
    @param name: the name of the parameter
    @type name: C{str}
    @param value: the value of the parameter
    @type value: C{str}
    '''
    QtGui.QStandardItem.__init__(self, unicode(value) if not isinstance(value, Binary) else str(value))
    self._name = name
    '''@ivar: the name of parameter '''
    self._value = value
    '''@ivar: the value of the parameter '''
    if isinstance(value, (str, unicode)) and value.find('\n') > -1:
      self.setSizeHint(QtCore.QSize(-1, 45))

  @property
  def name(self):
    return self._name

  @property
  def value(self):
    return self._value

  @value.setter
  def value(self, value):
    self._value = value
    self.setText(unicode(value) if not isinstance(value, Binary) else str(value))
    if isinstance(value, (str, unicode)) and value.find('\n') > -1:
      self.setSizeHint(QtCore.QSize(-1, 45))

  def type(self):
    return ParameterValueItem.ITEM_TYPE

  def data(self, role):
    if role == self.NAME_ROLE:
      return self.name
    elif role == self.VALUE_ROLE:
      return str(self.value)
    elif role == self.TYPE_ROLE:
      return str(type(self.value).replace('<type \'').replace('\'>'))
    else:
      return QtGui.QStandardItem.data(self, role)

  def __eq__(self, item):
    '''
    Compares the value of parameter.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return unicode(self.value) == item
    elif not (item is None):
      return unicode(self.value) == unicode(item.value)
    return False

  def __gt__(self, item):
    '''
    Compares the value of parameter.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return unicode(self.value) > item
    elif not (item is None):
      return unicode(self.value) > unicode(item.value)
    return False

class ParameterNameItem(QtGui.QStandardItem):
  '''
  The parameter item is stored in the parameter model. This class stores the name 
  and value of a parameter of ROS parameter server and shows the name.
  '''

  ITEM_TYPE = QtGui.QStandardItem.UserType + 38
  NAME_ROLE = QtCore.Qt.UserRole + 1
  VALUE_ROLE = QtCore.Qt.UserRole + 2
  TYPE_ROLE = QtCore.Qt.UserRole + 3

  def __init__(self, name, value, parent=None):
    '''
    Initialize the item object.
    @param name: the name of the parameter
    @type name: C{str}
    @param value: the value of the parameter
    @type value: C{str}
    '''
    QtGui.QStandardItem.__init__(self, name)
    self._name = name
    '''@ivar: the name of parameter '''
    self._value = value
    '''@ivar: the value of the parameter '''

  @property
  def name(self):
    return self._name

  @property
  def value(self):
    return self._value

  @value.setter
  def value(self, value):
    self._value = value
    self.setText(str(value))

  def type(self):
    return ParameterValueItem.ITEM_TYPE

  def data(self, role):
    if role == self.NAME_ROLE:
      return self.name
    elif role == self.VALUE_ROLE:
      return str(self.value)
    elif role == self.TYPE_ROLE:
      return str(type(self.value).replace('<type \'').replace('\'>'))
    else:
      return QtGui.QStandardItem.data(self, role)

  def __eq__(self, item):
    '''
    Compares the name of parameter.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.name.lower() == item.lower()
    elif not (item is None):
      return self.name.lower() == item.name.lower()
    return False

  def __gt__(self, item):
    '''
    Compares the name of parameter.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.name.lower() > item.lower()
    elif not (item is None):
      return self.name.lower() > item.name.lower()
    return False


class ParameterTypeItem(QtGui.QStandardItem):
  '''
  The parameter item is stored in the parameter model. This class stores the name 
  and value of a parameter of ROS parameter server and shows the name.
  '''

  ITEM_TYPE = QtGui.QStandardItem.UserType + 40
  NAME_ROLE = QtCore.Qt.UserRole + 1
  VALUE_ROLE = QtCore.Qt.UserRole + 2
  TYPE_ROLE = QtCore.Qt.UserRole + 3

  def __init__(self, name, value, parent=None):
    '''
    Initialize the item object.
    @param name: the name of the parameter
    @type name: C{str}
    @param value: the value of the parameter
    @type value: C{str}
    '''
    QtGui.QStandardItem.__init__(self, str(type(value)).replace("<type '", '').replace("'>", ''))
    self._name = name
    '''@ivar: the name of parameter '''
    self._value = value
    '''@ivar: the value of the parameter '''

  @property
  def name(self):
    return self._name

  @property
  def value(self):
    return self._value

  @value.setter
  def value(self, value):
    self._value = value
    self.setText(str(value))

  def type(self):
    return ParameterValueItem.ITEM_TYPE

  def data(self, role):
    if role == self.NAME_ROLE:
      return self.name
    elif role == self.VALUE_ROLE:
      return str(self.value)
    elif role == self.TYPE_ROLE:
      return str(type(self.value).replace('<type \'').replace('\'>'))
    else:
      return QtGui.QStandardItem.data(self, role)

  def __eq__(self, item):
    '''
    Compares the name of parameter.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.name.lower() == item.lower()
    elif not (item is None):
      return self.name.lower() == item.name.lower()
    return False

  def __gt__(self, item):
    '''
    Compares the name of parameter.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.name.lower() > item.lower()
    elif not (item is None):
      return self.name.lower() > item.name.lower()
    return False



class ParameterModel(QtGui.QStandardItemModel):
  '''
  The model to manage the list with parameter in ROS network.
  '''
  header = [('Parameter', 300),
            ('Type', 50),
            ('Value', -1)]
  '''@ivar: the list with columns C{[(name, width), ...]}'''
  
  def __init__(self):
    '''
    Creates a new list model.
    '''
    QtGui.QStandardItemModel.__init__(self)
    self.setColumnCount(len(ParameterModel.header))
    self.setHorizontalHeaderLabels([label for label, _ in ParameterModel.header])

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
    if index.column() == 2:
      return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEditable
    return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable

  def updateModelData(self, parameters):
    '''
    Updates the parameter list model. New parameter will be inserted in sorting 
    order. Not available parameter removed from the model.
    @param parameters: The dictionary with parameter 
    @type parameters: C{dict(parameter name : value)}
    '''
    parameter_names = parameters.keys()
    root = self.invisibleRootItem()
    # remove not available items
    for i in reversed(range(root.rowCount())):
      parameterItem = root.child(i)
      if not parameterItem.name in parameter_names:
        root.removeRow(i)
    # add new items
    for (name, value) in parameters.items():
      doAddItem = True
      for i in range(root.rowCount()):
        parameterItem = root.child(i)
        if (parameterItem == name):
          # update item
          parameterValueItem = root.child(i, 2)
          parameterValueItem.value = value
          doAddItem = False
          break
        elif (parameterItem > name):
          root.insertRow(i, self.createParameter(name, value))
          doAddItem = False
          break
      if doAddItem:
        root.appendRow(self.createParameter(name, value))

  def createParameter(self, name, value):
    '''
    Creates the list of the items. This list is used for the 
    visualization of the parameter as a table row.
    @param name: the parameter name
    @type name: C{str}
    @param value: the value of the parameter
    @type value: each value, that can be converted to C{str} using L{str()}
    @return: the list for the representation as a row
    @rtype: C{[L{ParameterNameItem}, L{ParameterValueItem}]}
    '''
    items = []
    item = ParameterNameItem(name, value)
    item.setEditable(False)
    items.append(item)
    item = ParameterTypeItem(name, value)
    item.setEditable(False)
    items.append(item)
    itemValue = ParameterValueItem(name, value)
    itemValue.setEditable(True)
    items.append(itemValue)
    return items
