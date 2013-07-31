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

from python_qt_binding import QtCore, QtGui

import sys
import time
import threading

import roslib
import rospy
import node_manager_fkie as nm

from parameter_handler import ParameterHandler

def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")

class MyComboBox(QtGui.QComboBox):

  remove_item_signal = QtCore.Signal(str)
  
  parameter_description = None

  def keyPressEvent(self, event):
    key_mod = QtGui.QApplication.keyboardModifiers()
    if key_mod & QtCore.Qt.ShiftModifier and (event.key() == QtCore.Qt.Key_Delete):
      try:
        if self.currentText():
          for i in range(self.count()):
            if self.currentText() == self.itemText(i):
              self.removeItem(i)
              self.remove_item_signal.emit(self.currentText())
              self.clearEditText()
      except:
        import traceback
        print traceback.format_exc()
    QtGui.QComboBox.keyPressEvent(self, event)

class ParameterDescription(object):
  '''
  Used for internal representation of the parameter in dialog.
  '''
  def __init__(self, name, msg_type, value=None, widget=None):
    self._name = name
    self._type = msg_type
    if isinstance(self._type, dict):
      self._type = 'dict'
    elif isinstance(self._type, list):
      self._type = 'list'
    self._value = value
    self._widget = widget
    self._base_type, self._is_array_type, self._array_length = roslib.msgs.parse_type(self._type)
    self._is_primitive_type =  self._base_type in roslib.msgs.PRIMITIVE_TYPES or self._base_type in ['int', 'float', 'time', 'duration']
    self._is_time_type = self._base_type in ['time', 'duration']
  
  def __repr__(self):
    return ''.join([self._name, ' [', self._type, ']'])
  
  def name(self):
    return self._name
  
  def setWidget(self, widget):
    self._widget = widget
    if not widget is None:
      widget.parameter_description = self
      self.addCachedValuesToWidget()
  
  def widget(self):
    return self._widget

  def fullName(self):
    result = self.name()
    widget = self._widget
    while not widget is None:
      if isinstance(widget, (MainBox, GroupBox, ArrayBox)):
        result = roslib.names.ns_join(widget.name, result)
      widget = widget.parent()
    return result

  def isArrayType(self):
    return self._is_array_type

  def arrayLength(self):
    return self._array_length

  def isPrimitiveType(self):
    return self._is_primitive_type

  def isTimeType(self):
    return self._is_time_type
  
  def baseType(self):
    return self._base_type
  
  def updateValueFromField(self):
    field = self.widget()
    result = ''
    if isinstance(field, QtGui.QCheckBox):
      result = repr(field.isChecked())
    elif isinstance(field, QtGui.QLineEdit):
      result = field.text()
    elif isinstance(field, QtGui.QComboBox):
      result = field.currentText()
    self.setValue(result)

  def setValue(self, value):
    error_str = ''
    try:
      if isinstance(value, (dict, list)):
        self._value = value
      elif value:
        nm.history().addParamCache(self.fullName(), value)
        if self.isArrayType():
          if 'int' in self.baseType():
            self._value = map(int, value.split(','))
          elif 'float' in self.baseType():
            self._value = map(float, value.split(','))
          elif 'bool' in self.baseType():
            self._value = map(str2bool, value.split(','))
          else:
            self._value = [ s.encode(sys.getfilesystemencoding()) for s in value.split(',')]
          if not self.arrayLength() is None and self.arrayLength() != len(self._value):
            raise Exception(''.join(["Field [", self.fullName(), "] has incorrect number of elements: ", str(len(self._value)), " != ", str(self.arrayLength())]))
        else:
          if 'int' in self.baseType():
            self._value = int(value)
          elif 'float' in self.baseType():
            self._value = float(value)
          elif 'bool' in self.baseType():
            if isinstance(value, bool):
              self._value = value
            else:
              self._value = str2bool(value)
          elif self.isTimeType():
            if value == 'now':
              self._value = 'now'
            else:
              try:
                val = float(value)
                secs = int(val)
                nsecs = int((val - secs) * 1000000000)
                self._value = {'secs': secs, 'nsecs': nsecs}
              except:
                self._value = {'secs': 0, 'nsecs': 0}
          else:
            self._value = value.encode(sys.getfilesystemencoding())
      else:
        if self.isArrayType():
          arr = []
          self._value = arr
        else:
          if 'int' in self.baseType():
            self._value = 0
          elif 'float' in self.baseType():
            self._value = 0.0
          elif 'bool' in self.baseType():
            self._value = False
          elif self.isTimeType():
            self._value = {'secs': 0, 'nsecs': 0}
          else:
            self._value = ''
      nm.history().addParamCache(self.fullName(), value)
    except Exception, e:
      raise Exception(''.join(["Error while set value '", unicode(value), "' for '", self.fullName(), "': ", str(e)]))
    return self._value

  def value(self):
    if not self.isPrimitiveType() and not self.widget() is None:
      return self.widget().value()
    elif self.isPrimitiveType():
      self.updateValueFromField()
      if self.isTimeType() and self._value == 'now':
        # FIX: rostopic does not support 'now' values in sub-headers
        t = time.time()
        return {'secs': int(t), 'nsecs': int((t-int(t))*1000000)}
    return self._value

  def removeCachedValue(self, value):
    nm.history().removeParamCache(self.fullName(), value)

  def createTypedWidget(self, parent):
    result = None
    if self.isPrimitiveType():
      value = self._value
      if 'bool' in self.baseType():
        result = QtGui.QCheckBox(parent=parent)
        result.setObjectName(self.name())
        if not isinstance(value, bool):
          value = str2bool(value)
        result.setChecked(value)
      else:
        result = MyComboBox(parent=parent)
        result.setObjectName(self.name())
        result.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed))
        result.setEditable(True)
        result.remove_item_signal.connect(self.removeCachedValue)
        items = []
        if isinstance(value, list):
          items[len(items):] = value
        else:
          if not value is None and value:
            items.append(unicode(value))
          elif self.isTimeType():
            items.append('now')
        result.addItems(items)
    else:
      if self.isArrayType():
        result = ArrayBox(self.name(), self._type, parent=parent)
      else:
        result = GroupBox(self.name(), self._type, parent=parent)
    return result

  def addCachedValuesToWidget(self):
    if isinstance(self.widget(), QtGui.QComboBox):
      values = nm.history().cachedParamValues(self.fullName())
      for i in range(self.widget().count()):
        try:
          values.remove(self.widget().itemText(i))
        except:
          pass
      if self.widget().count() == 0:
        values.insert(0, '')
      self.widget().addItems(values)



class MainBox(QtGui.QWidget):
  '''
  Groups the parameter without visualization of the group. It is the main widget.
  '''
  def __init__(self, name, type, collapsible=True, parent=None):
    QtGui.QWidget.__init__(self, parent)
    self.setObjectName(name)
    self.name = name
    self.type = type
    self.params = []
    self.collapsed = False
    self.parameter_description = None
    vLayout = QtGui.QVBoxLayout()
    vLayout.setSpacing(0)
    self.options_layout = QtGui.QHBoxLayout()
    self.param_widget = QtGui.QFrame()
    self.name_label = QtGui.QLabel(name)
    font = self.name_label.font()
    font.setBold(True)
    self.name_label.setFont(font)
    self.type_label = QtGui.QLabel(''.join([' (', type, ')']))

    if collapsible:
      self.hide_button = QtGui.QPushButton('-')
      self.hide_button.setFlat(True)
      self.hide_button.setMaximumSize(20,20)
      self.hide_button.clicked.connect(self._on_hide_clicked)
      self.options_layout.addWidget(self.hide_button)
      self.options_layout.addWidget(self.name_label)
      self.options_layout.addWidget(self.type_label)
      self.options_layout.addStretch()
 
      vLayout.addLayout(self.options_layout)

      self.param_widget.setFrameShape(QtGui.QFrame.Box)
      self.param_widget.setFrameShadow(QtGui.QFrame.Raised)

    boxLayout = QtGui.QFormLayout()
    boxLayout.setVerticalSpacing(0)
    self.param_widget.setLayout(boxLayout)
    vLayout.addWidget(self.param_widget)
    self.setLayout(vLayout)
    if type in ['std_msgs/Header']:
      self.setCollapsed(True)

  def setCollapsed(self, value):
    self.collapsed = value
    self.param_widget.setVisible(not value)
    self.hide_button.setText('+' if self.collapsed else '-')

  def _on_hide_clicked(self):
    self.setCollapsed(not self.collapsed)
#    self.param_widget.setVisible(not self.param_widget.isVisible())
#    vis = self.param_widget.isVisible()
#    self.hide_button.setText('-' if vis else '+')

  def createFieldFromValue(self, value):
    self.setUpdatesEnabled(False)
    try:
      if isinstance(value, dict):
        self._createFieldFromDict(value)
    finally:
      self.setUpdatesEnabled(True)

  def _createFieldFromDict(self, value, layout=None):
    if layout is None:
      layout = self.param_widget.layout()
    # sort the items: 1. header, 2. all premitives (sorted), 3. list, dict (sorted)
    all_params = []
    primitives = []
    komplex = []
    for name, (_type, val) in value.items():
      if _type in ['std_msgs/Header']:
        all_params.append((name, _type, val))
      elif isinstance(val, (dict, list)):
        komplex.append((name, _type, val))
      else:
        primitives.append((name, _type, val))
    all_params.extend(sorted(primitives))
    all_params.extend(sorted(komplex))
    
    # create widgets
    for name, _type, val in all_params:
      field = self.getField(name)
      if field is None:
        param_desc = ParameterDescription(name, _type, val)
        field = param_desc.createTypedWidget(self)
        param_desc.setWidget(field)
        self.params.append(param_desc)
        if isinstance(field, (GroupBox, ArrayBox)):
          field.createFieldFromValue(val)
          layout.addRow(field)
        else:
          label_name = name if _type == 'string' else ''.join([name, ' (', _type, ')'])
          label = QtGui.QLabel(label_name, self)
          label.setObjectName(''.join([name, '_label']))
          label.setBuddy(field)
          layout.addRow(label, field)
      else:
        if isinstance(field, (GroupBox, ArrayBox)):
          field.createFieldFromValue(val)
        else:
          raise Exception(''.join(["Parameter with name '", name, "' already exists!"]))

  def value(self):
    result = dict()
    for param in self.params:
      result[param.name()] = param.value()
    return result

  def getField(self, name):
    for child in self.children():
      if child.objectName() == name:
        return child
    return None
  
  def removeAllFields(self):
    '''
    Remove the references between parameter and corresponding widgets 
    (ComboBox, CheckBox, ..) and remove these widgets from layouts.
    '''
    for child in self.param_widget.children():
      if isinstance(child, MyComboBox):
        child.parameter_description.setWidget(None)
        self.params.remove(child.parameter_description)
      elif isinstance(child, MainBox):
        child.removeAllFields()
        self.param_widget.layout().removeWidget(child)

  def filter(self, arg):
    '''
    Hide the parameter input field, which label dosn't contains the C{arg}.
    @param arg: the filter text
    @type art: C{str}
    '''
    result = False
    for child in self.param_widget.children():
      if isinstance(child, (MainBox, GroupBox, ArrayBox)):
        show = not (child.objectName().lower().find(arg.lower()) == -1)
        show = show or child.filter(arg)
        # hide group, if no parameter are visible
        child.setVisible(show)
        if show:
          child.setCollapsed(False)
          result = True
      elif isinstance(child, (QtGui.QWidget)) and not isinstance(child, (QtGui.QLabel)) and  not isinstance(child, (QtGui.QFrame)):
        label = child.parentWidget().layout().labelForField(child)
        if not label is None:
          show = not (child.objectName().lower().find(arg.lower()) == -1) or not (child.currentText().lower().find(arg.lower()) == -1)
          # set the parent group visible if it is not visible
          if show and not child.parentWidget().isVisible():
            child.parentWidget().setVisible(show)
          label.setVisible(show)
          child.setVisible(show)
          if show:
            result = True
    return result

  def setVisible(self, arg):
    if arg and not self.parentWidget() is None and not self.parentWidget().isVisible():
      self.parentWidget().setVisible(arg)
    QtGui.QWidget.setVisible(self, arg)



class GroupBox(MainBox):
  '''
  Groups the parameter of a dictionary, struct or class using the group box for 
  visualization.
  '''
  def __init__(self, name, type, parent=None):
    MainBox.__init__(self, name, type, True, parent)
    self.setObjectName(name)



class ArrayEntry(MainBox):
  '''
  A part of the ArrayBox to represent the elements of a list.
  '''
  def __init__(self, index, type, parent=None):
#    QtGui.QFrame.__init__(self, parent)
    MainBox.__init__(self, ''.join(['#',str(index)]), type, True, parent)
    self.index = index
    self.setObjectName(''.join(['[', str(index), ']']))
    self.param_widget.setFrameShape(QtGui.QFrame.Box)
    self.param_widget.setFrameShadow(QtGui.QFrame.Plain)
    self.type_label.setVisible(False)
#    boxLayout = QtGui.QFormLayout()
#    boxLayout.setVerticalSpacing(0)
#    label = QtGui.QLabel(''.join(['[', str(index), ']']))
#    self.param_widget.layout().addRow(label)
#    self.setLayout(boxLayout)

  def value(self):
    result = dict()
    for param in self.params:
      result[param.name()] = param.value()
    return result


class ArrayBox(MainBox):
  '''
  Groups the parameter of a list.
  '''
  def __init__(self, name, type, parent=None):
    MainBox.__init__(self, name, type, True, parent)
    self._dynamic_value = None
    self._dynamic_widget = None
    self._dynamic_items_count = 0
    
  def addDynamicBox(self):
    self._dynamic_items_count = 0
    addButton = QtGui.QPushButton("+")
    addButton.setMaximumSize(25,25)
    addButton.clicked.connect(self._on_add_dynamic_entry)
    self.options_layout.addWidget(addButton)
    self.count_label = QtGui.QLabel('0')
    self.options_layout.addWidget(self.count_label)
    remButton = QtGui.QPushButton("-")
    remButton.setMaximumSize(25,25)
    remButton.clicked.connect(self._on_rem_dynamic_entry)
    self.options_layout.addWidget(remButton)
  
  def _on_add_dynamic_entry(self):
    self.setUpdatesEnabled(False)
    try:
      if not self._dynamic_value is None:
        for v in self._dynamic_value:
          if isinstance(v, dict):
            entry_frame = ArrayEntry(self._dynamic_items_count, self.type)
            self.param_widget.layout().addRow(entry_frame)
            entry_frame._createFieldFromDict(v)
            self._dynamic_items_count += 1
            self.count_label.setText(str(self._dynamic_items_count))
    finally:
      self.setUpdatesEnabled(True)

  def _on_rem_dynamic_entry(self):
    if self._dynamic_items_count > 0:
      self._dynamic_items_count -= 1
      item = self.param_widget.layout().takeAt(self._dynamic_items_count)
      self.param_widget.layout().removeItem(item)
      try:
        # remove the referenced parameter, too
        for child in item.widget().children():
          if isinstance(child, MyComboBox):
            child.parameter_description.setWidget(None)
            self.params.remove(child.parameter_description)
          elif isinstance(child, MainBox):
            child.removeAllFields()
            self.param_widget.layout().removeWidget(child)
            child.parameter_description.setWidget(None)
            self.params.remove(child.parameter_description)
        item.widget().setParent(None)
        del item
      except:
        import traceback
        print traceback.format_exc()
      self.count_label.setText(str(self._dynamic_items_count))

  def createFieldFromValue(self, value):
    self.setUpdatesEnabled(False)
    try:
      if isinstance(value, list):
        self.addDynamicBox()
        self._dynamic_value = value
    finally:
      self.setUpdatesEnabled(True)

  def value(self):
    result = list()
    for i in range(self.param_widget.layout().rowCount()):
      item = self.param_widget.layout().itemAt(i, QtGui.QFormLayout.SpanningRole)
      if item and isinstance(item.widget(), ArrayEntry):
        result.append(item.widget().value())
    return result


class ScrollArea(QtGui.QScrollArea):
  '''
  ScrollArea provides the maximal width of the internal widget.
  '''
  
  def viewportEvent(self, arg):
    if self.widget() and self.viewport().size().width() != self.widget().maximumWidth():
      self.widget().setMaximumWidth(self.viewport().size().width())
    return QtGui.QScrollArea.viewportEvent(self, arg)



class ParameterDialog(QtGui.QDialog):
  '''
  This dialog creates an input mask for the given parameter and their types.
  '''

  def __init__(self, params=dict(), buttons=QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok, parent=None):
    '''
    Creates an input dialog.
    @param params: a dictionary with parameter names and (type, values). 
    The C{value}, can be a primitive value, a list with values or parameter 
    dictionary to create groups. In this case the type is the name of the group.
    @type params: C{dict(str:(str, {value, [..], dict()}))}
    '''
    QtGui.QDialog.__init__(self, parent=parent)
    self.setObjectName(' - '.join(['ParameterDialog', str(params)]))

    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.verticalLayout.setContentsMargins(1, 1, 1, 1)
    # add filter row
    self.filter_frame = QtGui.QFrame(self)
    filterLayout = QtGui.QHBoxLayout(self.filter_frame)
    filterLayout.setContentsMargins(1, 1, 1, 1)
    label = QtGui.QLabel("Filter:", self.filter_frame)
    self.filter_field = QtGui.QLineEdit(self.filter_frame)
    filterLayout.addWidget(label)
    filterLayout.addWidget(self.filter_field)
    self.filter_field.textChanged.connect(self._on_filter_changed)
    self.filter_visible = True

    self.verticalLayout.addWidget(self.filter_frame)

    # create area for the parameter
    self.scrollArea = scrollArea = ScrollArea(self);
    scrollArea.setObjectName("scrollArea")
    scrollArea.setWidgetResizable(True)
    self.content = MainBox('/', 'str', False, self)
    scrollArea.setWidget(self.content)
    self.verticalLayout.addWidget(scrollArea)

    # add info text field
    self.info_field = QtGui.QTextEdit(self)
    self.info_field.setVisible(False)
    palette = QtGui.QPalette()
    brush = QtGui.QBrush(QtGui.QColor(255, 254, 242))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
    brush = QtGui.QBrush(QtGui.QColor(255, 254, 242))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
    brush = QtGui.QBrush(QtGui.QColor(244, 244, 244))
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
    self.info_field.setPalette(palette)
    self.info_field.setFrameShadow(QtGui.QFrame.Plain)
    self.info_field.setReadOnly(True)
    self.info_field.setTextInteractionFlags(QtCore.Qt.LinksAccessibleByKeyboard|QtCore.Qt.LinksAccessibleByMouse|QtCore.Qt.TextBrowserInteraction|QtCore.Qt.TextSelectableByKeyboard|QtCore.Qt.TextSelectableByMouse)
    self.info_field.setObjectName("dialog_info_field")
    self.verticalLayout.addWidget(self.info_field)

    # create buttons
    self.buttonBox = QtGui.QDialogButtonBox(self)
    self.buttonBox.setObjectName("buttonBox")
    self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
    self.buttonBox.setStandardButtons(buttons)
    self.buttonBox.accepted.connect(self.accept)
    self.buttonBox.rejected.connect(self.reject)
    self.verticalLayout.addWidget(self.buttonBox)

    # set the input fields
    if params:
      self.content.createFieldFromValue(params)
      self.setInfoActive(False)

    if self.filter_frame.isVisible():
      self.filter_field.setFocus()
#    print '=============== create', self.objectName()
#
  def __del__(self):
#    print "************ destroy", self.objectName()
    self.content.removeAllFields()

  def _on_filter_changed(self):
    self.content.filter(self.filter_field.text())

  def setFilterVisible(self, val):
    '''
    Shows or hides the filter row.
    '''
    self.filter_visible = val
    self.filter_frame.setVisible(val&self.scrollArea.isHidden())

  def add_warning(self, message):
    label = QtGui.QLabel()
    label.setWordWrap(True)
    label.setText(''.join(["<font color='red'>Warning!\n", message, "</font>"]))
    self.verticalLayout.insertWidget(1, label)

  def setText(self, text):
    '''
    Adds a label to the dialog's layout and shows the given text.
    @param text: the text to add to the dialog
    @type text: C{str}
    '''
    self.info_field.setText(text)
    self.setInfoActive(True)

  def setInfoActive(self, val):
    '''
    Activates or deactivates the info field of this dialog. If info field is
    activated, the filter frame and the input field are deactivated.
    @type val: C{bool} 
    '''
    if val and self.info_field.isHidden():
      self.filter_frame.setVisible(False&self.filter_visible)
      self.scrollArea.setVisible(False)
      self.info_field.setVisible(True)
    elif not val and self.scrollArea.isHidden():
      self.filter_frame.setVisible(True&self.filter_visible)
      self.scrollArea.setVisible(True)
      self.info_field.setVisible(False)
      if self.filter_frame.isVisible():
        self.filter_field.setFocus()

  def setFocusField(self, field_label):
    field = self.content.getField(field_label)
    if not field is None:
      field.setFocus()

  def getKeywords(self):
    '''
    @returns: a directory with parameter and value for all entered fields.
    @rtype: C{dict(str(param) : str(value))}
    '''
    return self.content.value()

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%% close handling                        %%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def accept(self):
    self.setResult(QtGui.QDialog.Accepted)
    self.hide()
  
  def reject(self):
    self.setResult(QtGui.QDialog.Rejected)
    self.hide()

  def hideEvent(self, event):
    self.close()

  def closeEvent (self, event):
    '''
    Test the open files for changes and save this if needed.
    '''
    self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
    QtGui.QDialog.closeEvent(self, event)



class MasterParameterDialog(ParameterDialog):
  '''
  This dialog is an extension to the L{ParameterDialog}. The parameter and their 
  values are requested from the ROS master parameter server. The requests are 
  threaded and allows the also threaded changed of ROS parameter assigned to 
  given namespace.
  '''
  
  def __init__(self, masteruri, ns='/', parent=None):
    '''
    @param masteruri: if the master uri is not None, the parameter are retrieved from ROS parameter server.
    @type masteruri: C{str}
    @param ns: namespace of the parameter retrieved from the ROS parameter server.
    @type ns: C{str}
    '''
    ParameterDialog.__init__(self, dict(), parent=parent)
    self.masteruri = masteruri
    self.ns = ns
    self.is_delivered = False
    self.is_send = False
    self.mIcon = QtGui.QIcon(":/icons/default_cfg.png")
    self.setWindowIcon(self.mIcon)
    self.resize(450,300)
    self.add_new_button = QtGui.QPushButton(self.tr("&Add"))
    self.add_new_button.clicked.connect(self._on_add_parameter)
    self.buttonBox.addButton(self.add_new_button, QtGui.QDialogButtonBox.ActionRole)
#    self.apply_button = QtGui.QPushButton(self.tr("&Ok"))
#    self.apply_button.clicked.connect(self._on_apply)
#    self.buttonBox.addButton(self.apply_button, QtGui.QDialogButtonBox.ApplyRole)
#    self.buttonBox.accepted.connect(self._on_apply)
    self.setText(' '.join(['Obtaining parameters from the parameter server', masteruri, '...']))
    self.parameterHandler = ParameterHandler()
    self.parameterHandler.parameter_list_signal.connect(self._on_param_list)
    self.parameterHandler.parameter_values_signal.connect(self._on_param_values)
    self.parameterHandler.delivery_result_signal.connect(self._on_delivered_values)
    self.parameterHandler.requestParameterList(masteruri, ns)
#    self.apply_button.setFocus(QtCore.Qt.OtherFocusReason)

  def accept(self):
    if not self.masteruri is None and not self.is_send:
      try:
        params = self.getKeywords()
        ros_params = dict()
        for p,v in params.items():
          ros_params[roslib.names.ns_join(self.ns, p)] = v
        if ros_params:
          self.is_send = True
          self.setText('Send the parameter into server...')
          self.parameterHandler.deliverParameter(self.masteruri, ros_params)
      except Exception, e:
        QtGui.QMessageBox.warning(self, self.tr("Warning"), str(e), QtGui.QMessageBox.Ok)
    elif self.masteruri is None:
      QtGui.QMessageBox.warning(self, self.tr("Error"), 'Invalid ROS master URI', QtGui.QMessageBox.Ok)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%          ROS parameter handling       %%%%%%%%%%%%%%%%%%%%%
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def _on_add_parameter(self):
    params_arg = {'namespace' : ('string', self.ns), 'name' : ('string', ''), 'type' : ('string', ['string', 'int', 'float', 'bool']), 'value' : ('string', '') }
    dia = ParameterDialog(params_arg)
    dia.setWindowTitle('Add new parameter')
    dia.resize(360,150)
    dia.setFilterVisible(False)
    if dia.exec_():
      try:
        params = dia.getKeywords()
        if params['name']:
          if params['type'] == 'int':
            value = int(params['value'])
          elif params['type'] == 'float':
            value = float(params['value'])
          elif params['type'] == 'bool':
            value = str2bool(params['value'])
          else:
            value = params['value']
          self._on_param_values(self.masteruri, 1, '', {roslib.names.ns_join(params['namespace'], params['name']) : (1, '', value)})
        else:
          QtGui.QMessageBox.warning(self, self.tr("Warning"), 'Empty name is not valid!', QtGui.QMessageBox.Ok)
      except ValueError, e:
        QtGui.QMessageBox.warning(self, self.tr("Warning"), unicode(e), QtGui.QMessageBox.Ok)

  def _on_param_list(self, masteruri, code, msg, params):
    '''
    @param masteruri: The URI of the ROS parameter server
    @type masteruri: C{str}
    @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
    @type code: C{int}
    @param msg: The message of the result. 
    @type msg: C{str}
    @param params: The list the parameter names.
    @type param: C{[str]}
    '''
    if code == 1:
      params.sort()
      self.parameterHandler.requestParameterValues(masteruri, params)
    else:
      self.setText(msg)
      
  def _on_param_values(self, masteruri, code, msg, params):
    '''
    @param masteruri: The URI of the ROS parameter server
    @type masteruri: C{str}
    @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
    @type code: C{int}
    @param msg: The message of the result. 
    @type msg: C{str}
    @param params: The dictionary the parameter names and request result.
    @type param: C{dict(paramName : (code, statusMessage, parameterValue))}
    '''
    if code == 1:
      dia_params = dict()
      for p, (code_n, msg_n, val) in params.items():
        if code_n != 1:
          val = ''
        type_str = 'string'
        value = val
        if isinstance(val, bool):
          type_str = 'bool'
        elif isinstance(val, int):
          type_str = 'int'
        elif isinstance(val, float):
          type_str = 'float'
        elif isinstance(val, list) or isinstance(val, dict):
          value = unicode(val)
        param = p.replace(self.ns, '')
        names_sep = param.split(roslib.names.SEP)
        param_name = names_sep.pop()
        if names_sep:
          group = dia_params
          for n in names_sep:
            group_name = n
            if group.has_key(group_name):
              group = group[group_name][1]
            else:
              tmp_dict = dict()
              group[group_name] = (n, tmp_dict)
              group = tmp_dict
          group[param_name] = (type_str, value)
        else:
          dia_params[param_name] = (type_str, value)
      try:
        self.content.createFieldFromValue(dia_params)
        self.setInfoActive(False)
      except Exception, e:
        QtGui.QMessageBox.warning(self, self.tr("Warning"), unicode(e), QtGui.QMessageBox.Ok)
    else:
      self.setText(msg)

  def _on_delivered_values(self, masteruri, code, msg, params):
    '''
    @param masteruri: The URI of the ROS parameter server
    @type masteruri: C{str}
    @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
    @type code: C{int}
    @param msg: The message of the result. 
    @type msg: C{str}
    @param params: The dictionary the parameter names and request result.
    @type param: C{dict(paramName : (code, statusMessage, parameterValue))}
    '''
    self.is_delivered = True
    errmsg = ''
    if code == 1:
      for p, (code_n, msg, val) in params.items():
        if code_n != 1:
          errmsg = '\n'.join([errmsg, msg])
    else:
      errmsg = msg if msg else 'Unknown error on set parameter'
    if errmsg:
      QtGui.QMessageBox.warning(self, self.tr("Warning"), errmsg, QtGui.QMessageBox.Ok)
      self.is_delivered = False
      self.is_send = False
      self.setInfoActive(False)
    if self.is_delivered:
      self.close()



class ServiceDialog(ParameterDialog):
  '''
  Adds a support for calling a service to the L{ParameterDialog}. The needed 
  input fields are created from the service request message type. The service 
  call is executed in a thread to avoid blocking GUI.
  '''
  service_resp_signal = QtCore.Signal(str, str)

  def __init__(self, service, parent=None):
    '''
    @param service: Service to call.
    @type service: L{ServiceInfo}
    '''
    self.service = service
    slots = service.get_service_class(True)._request_class.__slots__
    types = service.get_service_class()._request_class._slot_types
    ParameterDialog.__init__(self, self._params_from_slots(slots, types), buttons=QtGui.QDialogButtonBox.Close, parent=parent)
    self.setWindowTitle(''.join(['Call ', service.name]))
    self.service_resp_signal.connect(self._handle_resp)
    self.resize(450,300)
    if not slots:
      self.setText(''.join(['Wait for response ...']))
      thread = threading.Thread(target=self._callService)
      thread.setDaemon(True)
      thread.start()
    else:
      self.call_service_button = QtGui.QPushButton(self.tr("&Call"))
      self.call_service_button.clicked.connect(self._on_call_service)
      self.buttonBox.addButton(self.call_service_button, QtGui.QDialogButtonBox.ActionRole)
      self.hide_button = QtGui.QPushButton(self.tr("&Hide/Show output"))
      self.hide_button.clicked.connect(self._on_hide_output)
      self.buttonBox.addButton(self.hide_button, QtGui.QDialogButtonBox.ActionRole)
      self.hide_button.setVisible(False)

  def _on_hide_output(self):
    self.setInfoActive(not self.info_field.isVisible())

  def _on_call_service(self):
    try:
      self.hide_button.setVisible(True)
      params = self.getKeywords()
      self.setText(''.join(['Wait for response ...']))
      thread = threading.Thread(target=self._callService, args=((params,)))
      thread.setDaemon(True)
      thread.start()
    except Exception, e:
      rospy.logwarn("Error while reading parameter for %s service: %s", str(self.service.name), unicode(e))
      self.setText(''.join(['Error while reading parameter:\n', unicode(e)]))

  def _callService(self, params={}):
    req = unicode(params) if params else ''
    try:
      req, resp = nm.starter().callService(self.service.uri, self.service.name, self.service.get_service_class(), [params])
      self.service_resp_signal.emit(str(req), str(resp))
    except Exception, e:
      import traceback
      print traceback.format_exc()
      rospy.logwarn("Error while call service '%s': %s", str(self.service.name), str(e))
      self.service_resp_signal.emit(unicode(req), unicode(e))

  @classmethod
  def _params_from_slots(cls, slots, types):
    result = dict()
    for slot, msg_type in zip(slots, types):
      base_type, is_array, array_length = roslib.msgs.parse_type(msg_type)
      if base_type in roslib.msgs.PRIMITIVE_TYPES or base_type in ['time', 'duration']:
        result[slot] = (msg_type, 'now' if base_type in ['time', 'duration'] else '')
      else:
        try:
          list_msg_class = roslib.message.get_message_class(base_type)
          subresult = cls._params_from_slots(list_msg_class.__slots__, list_msg_class._slot_types)
          result[slot] = (msg_type, [subresult] if is_array else subresult)
        except ValueError, e:
          import traceback
          print traceback.format_exc()
          rospy.logwarn("Error while parse message type '%s': %s", str(msg_type), str(e))
    return result

  def _handle_resp(self, req, resp):
    self.setWindowTitle(''.join(['Request / Response of ', self.service.name]))
    self.setText('\n'.join([unicode(req), '---', unicode(resp)]))
