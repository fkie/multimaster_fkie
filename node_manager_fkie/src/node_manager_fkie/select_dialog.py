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

import roslib
import rospy
from parameter_dialog import ScrollArea

class SelectDialog(QtGui.QDialog):
  '''
  This dialog creates an input mask for a string list and return selected entries.
  '''

  def __init__(self, input=list(), buttons=QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok, exclusive=False, parent=None):
    '''
    Creates an input dialog.
    @param input: a list with strings
    @type input: C{list()}
    '''
    QtGui.QDialog.__init__(self, parent=parent)
    self.setObjectName(' - '.join(['SelectDialog', str(input)]))

    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.verticalLayout.setContentsMargins(1, 1, 1, 1)
    
    # create area for the parameter
    self.scrollArea = scrollArea = QtGui.QScrollArea(self);
    scrollArea.setObjectName("scrollArea")
    scrollArea.setWidgetResizable(True)
    self.content = MainBox(self)
    scrollArea.setWidget(self.content)
    self.verticalLayout.addWidget(scrollArea)
    
    # add select all option
    if not exclusive:
      options = QtGui.QWidget(self)
      hLayout = QtGui.QHBoxLayout(options)
      hLayout.setContentsMargins(1, 1, 1, 1)
      self.select_all_checkbox = QtGui.QCheckBox('all')
  #    self.select_all_checkbox.setTristate(True)
      try:
        self.select_all_checkbox.setCheckState(QtCore.Qt.CheckState.Checked)#PySide
      except:
        self.select_all_checkbox.setCheckState(QtCore.Qt.Checked)  #PyQt4
      self.select_all_checkbox.toggled.connect(self._on_select_all_checkbox_toggled)
      hLayout.addWidget(self.select_all_checkbox)
      # add spacer
      spacerItem = QtGui.QSpacerItem(515, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
      hLayout.addItem(spacerItem)
      self.verticalLayout.addWidget(options)

    # create buttons
    self.buttonBox = QtGui.QDialogButtonBox(self)
    self.buttonBox.setObjectName("buttonBox")
    self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
    self.buttonBox.setStandardButtons(buttons)
    self.buttonBox.accepted.connect(self.accept)
    self.buttonBox.rejected.connect(self.reject)
    self.verticalLayout.addWidget(self.buttonBox)

    # set the input fields
    if input:
      self.content.createFieldsFromValues(input, exclusive)

#    print '=============== create', self.objectName()
#
#  def __del__(self):
#    print "************ destroy", self.objectName()

  def _on_select_all_checkbox_toggled(self, state):
    self.content.setState(state)

  def _on_filter_changed(self):
    self.content.filter(self.filter_field.text())

  def getSelected(self):
    return self.content.getSelected()

  @staticmethod
  def getValue(title, input=list(), exclusive=False, parent=None):
    selectDia = SelectDialog(input, exclusive=exclusive, parent=parent)
    selectDia.setWindowTitle(title)
    selectDia.resize(480, 256)
    if selectDia.exec_():
      return selectDia.getSelected()
    return list()


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


class MainBox(QtGui.QWidget):
  '''
  A widget with entries.
  '''
  def __init__(self, parent=None):
    QtGui.QWidget.__init__(self, parent)
    self.setObjectName("MainBox")
    boxLayout = QtGui.QFormLayout()
    boxLayout.setVerticalSpacing(0)
    self.box_group = QtGui.QButtonGroup(self)
    self.setLayout(boxLayout)

  def createFieldsFromValues(self, values, exclusive=False):
    self.setUpdatesEnabled(False)
    self.box_group.setExclusive(exclusive)
    try:
      if isinstance(values, list):
        for v in values:
          checkbox = QtGui.QCheckBox(v)
          checkbox.setObjectName(v)
          self.box_group.addButton(checkbox)
          self.layout().addRow(checkbox)
          try:
            checkbox.setCheckState(QtCore.Qt.CheckState.Checked)#PySide
          except:
            checkbox.setCheckState(QtCore.Qt.Checked)#PyQt4
    finally:
      self.setUpdatesEnabled(True)

  def getSelected(self):
    result = list()
    for i in range(self.layout().count()):
      item = self.layout().itemAt(i).widget()
      if isinstance(item, QtGui.QCheckBox):
        if item.isChecked():
          result.append(item.text())
    return result
  
  def setState(self, state):
    for i in range(self.layout().count()):
      item = self.layout().itemAt(i).widget()
      if isinstance(item, QtGui.QCheckBox):
        try:
          item.setCheckState(QtCore.Qt.CheckState.Checked if state else QtCore.Qt.CheckState.Unchecked)#PySide
        except:
          item.setCheckState(QtCore.Qt.Checked if state else QtCore.Qt.Unchecked)#PyQt4
