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
from python_qt_binding import QtCore, QtGui


class SelectDialog(QtGui.QDialog):
  '''
  This dialog creates an input mask for a string list and return selected entries.
  '''

  def __init__(self, items=list(), buttons=QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok, exclusive=False, preselect_all=False, title='', description='', icon='', parent=None, select_if_single=True):
    '''
    Creates an input dialog.
    @param items: a list with strings
    @type items: C{list()}
    '''
    QtGui.QDialog.__init__(self, parent=parent)
    self.setObjectName(' - '.join(['SelectDialog', str(items)]))

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
    self.verticalLayout.addWidget(self.filter_frame)

    if description:
      self.description_frame = QtGui.QFrame(self)
      descriptionLayout = QtGui.QHBoxLayout(self.description_frame)
#      descriptionLayout.setContentsMargins(1, 1, 1, 1)
      if icon:
        self.icon_label = QtGui.QLabel(self.description_frame)
        self.icon_label.setSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        self.icon_label.setPixmap(QtGui.QPixmap(icon).scaled(30, 30, QtCore.Qt.KeepAspectRatio))
        descriptionLayout.addWidget(self.icon_label)
      self.description_label = QtGui.QLabel(self.description_frame)
      self.description_label.setWordWrap(True)
      self.description_label.setText(description)
      descriptionLayout.addWidget(self.description_label)
      self.verticalLayout.addWidget(self.description_frame)

    # create area for the parameter
    self.scrollArea = scrollArea = QtGui.QScrollArea(self);
    self.scrollArea.setFocusPolicy(QtCore.Qt.NoFocus)
    scrollArea.setObjectName("scrollArea")
    scrollArea.setWidgetResizable(True)
    self.content = MainBox(self)
    scrollArea.setWidget(self.content)
    self.verticalLayout.addWidget(scrollArea)

    # add select all option
    if not exclusive:
      self._ignore_next_toggle = False
      options = QtGui.QWidget(self)
      hLayout = QtGui.QHBoxLayout(options)
      hLayout.setContentsMargins(1, 1, 1, 1)
      self.select_all_checkbox = QtGui.QCheckBox('all')
      self.select_all_checkbox.setTristate(True)
      self.select_all_checkbox.stateChanged.connect(self._on_select_all_checkbox_stateChanged)
      hLayout.addWidget(self.select_all_checkbox)
      self.content.toggled.connect(self._on_main_toggle)
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
    if items:
      self.content.createFieldsFromValues(items, exclusive)
      if (select_if_single and len(items) == 1) or preselect_all:
        self.select_all_checkbox.setCheckState(QtCore.Qt.Checked)

    if not items or len(items) < 11:
      self.filter_frame.setVisible(False)
#    print '=============== create', self.objectName()
#
#  def __del__(self):
#    print "************ destroy", self.objectName()

  def _on_main_toggle(self, state):
    self._ignore_next_toggle = state != self.select_all_checkbox.checkState()
    self.select_all_checkbox.setCheckState(state)

  def _on_select_all_checkbox_stateChanged(self, state):
    if not self._ignore_next_toggle:
      self.content.setState(state)
    self._ignore_next_toggle = False

  def _on_filter_changed(self):
    self.content.filter(self.filter_field.text())

  def getSelected(self):
    return self.content.getSelected()

  @staticmethod
  def getValue(title, description='', items=list(), exclusive=False, preselect_all=False, icon='', parent=None, select_if_single=True):
    selectDia = SelectDialog(items, exclusive=exclusive, preselect_all=preselect_all, description=description, icon=icon, parent=parent, select_if_single=select_if_single)
    selectDia.setWindowTitle(title)
    selectDia.resize(480, 256)
    if selectDia.exec_():
      return selectDia.getSelected(), True
    return list(), False


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
  
  toggled = QtCore.Signal(QtCore.Qt.CheckState)
  
  def __init__(self, parent=None):
    QtGui.QWidget.__init__(self, parent)
    self.setObjectName("MainBox")
    self.__on_intern_change = False
    boxLayout = QtGui.QFormLayout()
    boxLayout.setVerticalSpacing(0)
    self.setLayout(boxLayout)

  def createFieldsFromValues(self, values, exclusive=False):
    self.setUpdatesEnabled(False)
    try:
      if isinstance(values, list):
        for v in values:
          checkbox = QtGui.QCheckBox(v)
          checkbox.toggled.connect(self._on_checkbox_toggled)
          checkbox.setObjectName(v)
          checkbox.setAutoExclusive(exclusive)
          self.layout().addRow(checkbox)
    finally:
      self.setUpdatesEnabled(True)

  def _on_checkbox_toggled(self):
    if not self.__on_intern_change:
      l = self.getSelected()
      if len(l) == 0:
        self.toggled.emit(QtCore.Qt.Unchecked)
      elif len(l) == self.layout().count():
        self.toggled.emit(QtCore.Qt.Checked)
      else:
        self.toggled.emit(QtCore.Qt.PartiallyChecked)

  def filter(self, arg):
    '''
    Hide the parameter input field, which label dosn't contains the C{arg}.
    @param arg: the filter text
    @type art: C{str}
    '''
    for i in range(self.layout().count()):
      item = self.layout().itemAt(i).widget()
      if isinstance(item, QtGui.QCheckBox):
        new_state = (not re.search(arg, item.objectName()) is None)
        item.setVisible(new_state)
        if new_state:
          self._on_checkbox_toggled()

  def getSelected(self):
    result = list()
    for i in range(self.layout().count()):
      item = self.layout().itemAt(i).widget()
      if isinstance(item, QtGui.QCheckBox):
        if item.isChecked():
          result.append(item.text())
    return result
  
  def setState(self, state):
    self.__on_intern_change = True
    for i in range(self.layout().count()):
      item = self.layout().itemAt(i).widget()
      if isinstance(item, QtGui.QCheckBox):
        if state == QtCore.Qt.Checked:
          item.setCheckState(QtCore.Qt.Checked)
        elif state == QtCore.Qt.Unchecked:
          item.setCheckState(QtCore.Qt.Unchecked)
        elif state == QtCore.Qt.PartiallyChecked and item.isVisible():
          item.setCheckState(QtCore.Qt.Checked)
    self.__on_intern_change = False
    self._on_checkbox_toggled()
