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
#  * Neither the name of I Heart Engineering nor the names of its
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

from PySide import QtCore, QtGui

import roslib
import rospy
import node_manager_fkie as nm

class ParameterDialog(QtGui.QDialog):
  '''
  This dialog creates an input mask for the given slots and their types.
  '''

  def __init__(self, slots, types, values=dict(), buttons=QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok, parent=None):
    '''
    Creates an input dialog.
    @param slots: a list with  slots to enter
    @type slots: C{[str, ...]}
    @param types: a list with  types of the slots
    @type types: C{[str, ...]}
    @param values: a dictionary with slots and their values
    @type values: C{dict((str:[..]), ...)}
    '''
    QtGui.QDialog.__init__(self, parent)
    self.setObjectName("ParameterDialog")
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.verticalLayout.setContentsMargins(1, 1, 1, 1)
    
    scrollArea = QtGui.QScrollArea(self);
    scrollArea.setObjectName("scrollArea")
    scrollArea.setWidgetResizable(True)
    self.content = QtGui.QWidget()
    self.content.setObjectName("scrollAreaWidgetContents")
    self.contentLayout = QtGui.QFormLayout(self.content)
#    self.contentLayout.setVerticalSpacing(0)
#    self.contentLayout.setHorizontalSpacing(1)
    scrollArea.setWidget(self.content)
  
    self.verticalLayout.addWidget(scrollArea);

    self.params = list()
#    self.contentLayout.setFieldGrowthPolicy(QtGui.QFormLayout.ExpandingFieldsGrow)
    self._insertItems(self.content, self.contentLayout, slots, types, values)
    if self.params:
      self.params[0][2].setFocus(QtCore.Qt.OtherFocusReason)
#    self.content.setLayout(self.contentLayout)
    self.buttonBox = QtGui.QDialogButtonBox(self)
    self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
    self.buttonBox.setStandardButtons(buttons)
    self.buttonBox.setObjectName("buttonBox")
    self.verticalLayout.addWidget(self.buttonBox)

    QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), self.accept)
    QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), self.reject)
    QtCore.QMetaObject.connectSlotsByName(self)

  def setText(self, text):
    '''
    Adds a label to the dialog's layout and shows the given text.
    @param text: the text to add to the dialog
    @type text: C{str}
    '''
    label = QtGui.QLabel(text, self.content)
    self.contentLayout.addRow(label)

  def _insertItems(self, parent, layout, names, types, values=dict()):
    '''
    Adds input fields to the layout of the dialog.
    @param values: a dictionary with slots and their values
    @type values: C{dict((str:[]), ...)}
    '''
    for name, type in zip(names, types):
      base_type = roslib.msgs.base_msg_type(type)
      if base_type in roslib.msgs.PRIMITIVE_TYPES:
        if base_type == 'bool':
          field = QtGui.QCheckBox(parent)
        else:
          if values.has_key(name):
            field = QtGui.QComboBox(parent)
            field.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed))
            field.setEditable(True)
            field.addItems(values[name])
          else:
            field = QtGui.QLineEdit(parent)
            if nm.PARAM_CACHE.has_key(name):
              field.setText(nm.PARAM_CACHE[name])
        field.setObjectName(name);
        label = QtGui.QLabel(''.join([name, ' (', type, ')']), parent)
        label.setObjectName(''.join([name, '_label']))
        label.setBuddy(field)
        layout.addRow(label, field)
        self.params.append((name, type, field))
      else:
        try:
          if base_type == 'time':
            QtGui.QMessageBox.information(self, self.tr("sorry"),
                                '<time> parameter is not supported yet! This parameter will be ignored!',
                                QtGui.QMessageBox.Ok)
          else:
            list_msg_class =roslib.message.get_message_class(base_type)
            box = QtGui.QGroupBox(type, parent)
            boxLayout = QtGui.QFormLayout(box)
            self._insertItems(box, boxLayout, list_msg_class.__slots__, list_msg_class._slot_types)
            box.setLayout(boxLayout)
            layout.addRow(box)
        except ValueError, e:
          print "ERROR", e
          pass

  def getKeywords(self):
    '''
    @returns: a directory with parameter and value for all entered fields.
    @rtype: C{dict(str(param) : str(value))}
    '''
    result = {}
    for key, type, value in self.params:
      if isinstance(value, QtGui.QCheckBox):
        result[key] = value.isChecked()
      else:
        test = ''
        if isinstance(value, QtGui.QLineEdit):
          text = value.text()
        elif isinstance(value, QtGui.QComboBox):
          text = value.currentText()
        if text:
          nm.PARAM_CACHE[key] = text
          if 'int' in type:
            result[key] = int(text)
          elif 'float' in type:
            result[key] = float(text)
          else:
            result[key] = str(text)
    return result