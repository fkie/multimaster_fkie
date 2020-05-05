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



from python_qt_binding.QtCore import Qt, Signal, QPoint, QSize
try:
    from python_qt_binding.QtGui import QCheckBox, QDialog, QFrame, QDialogButtonBox, QLabel, QLineEdit, QScrollArea, QWidget
    from python_qt_binding.QtGui import QFormLayout, QHBoxLayout, QVBoxLayout, QSizePolicy, QSpacerItem
except Exception:
    from python_qt_binding.QtWidgets import QCheckBox, QDialog, QFrame, QDialogButtonBox, QLabel, QLineEdit, QScrollArea, QWidget
    from python_qt_binding.QtWidgets import QFormLayout, QHBoxLayout, QVBoxLayout, QSizePolicy, QSpacerItem
from python_qt_binding.QtGui import QPixmap
import re

import threading
import fkie_node_manager as nm
from fkie_node_manager_daemon.common import utf8
from fkie_node_manager.editor.line_edit import EnhancedLineEdit


class SelectDialog(QDialog):
    '''
    This dialog creates an input mask for a string list and return selected entries.
    '''

    MODAL_DIALOG = None

    def __init__(self, items=list(), buttons=QDialogButtonBox.Cancel | QDialogButtonBox.Ok, exclusive=False,
                 preselect_all=False, title='', description='', icon='', parent=None, select_if_single=True,
                 checkitem1='', checkitem2='', closein=0, store_geometry=''):
        '''
        Creates an input dialog.
        @param items: a list with strings
        @type items: C{list()}
        '''
        QDialog.__init__(self, parent=parent)
        self.setObjectName(' - '.join(['SelectDialog', utf8(items)]))

        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(3, 3, 3, 3)

        # add filter row
        self.filter_field = EnhancedLineEdit(self)
        self.filter_field.setPlaceholderText("filter")
        self.filter_field.textChanged.connect(self._on_filter_changed)
        self.verticalLayout.addWidget(self.filter_field)

        if description:
            self.description_frame = QFrame(self)
            descriptionLayout = QHBoxLayout(self.description_frame)
#      descriptionLayout.setContentsMargins(1, 1, 1, 1)
            if icon:
                self.icon_label = QLabel(self.description_frame)
                self.icon_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
                self.icon_label.setPixmap(QPixmap(icon).scaled(30, 30, Qt.KeepAspectRatio))
                descriptionLayout.addWidget(self.icon_label)
            self.description_label = QLabel(self.description_frame)
            self.description_label.setWordWrap(True)
            self.description_label.setText(description)
            descriptionLayout.addWidget(self.description_label)
            self.verticalLayout.addWidget(self.description_frame)

        # create area for the parameter
        self.content = MainBox(self)
        if items:
            self.scroll_area = QScrollArea(self)
            self.scroll_area.setFocusPolicy(Qt.NoFocus)
            self.scroll_area.setObjectName("scroll_area")
            self.scroll_area.setWidgetResizable(True)
            self.scroll_area.setWidget(self.content)
            self.verticalLayout.addWidget(self.scroll_area)

        self.checkitem1 = checkitem1
        self.checkitem1_result = False
        self.checkitem2 = checkitem2
        self.checkitem2_result = False

        # add select all option
        if not exclusive and items:
            self._ignore_next_toggle = False
            self.select_all_checkbox = QCheckBox('all entries')
            self.select_all_checkbox.setTristate(True)
            self.select_all_checkbox.stateChanged.connect(self._on_select_all_checkbox_stateChanged)
            self.verticalLayout.addWidget(self.select_all_checkbox)
            self.content.toggled.connect(self._on_main_toggle)
        if self.checkitem1:
            self.checkitem1_checkbox = QCheckBox(self.checkitem1)
            self.checkitem1_checkbox.stateChanged.connect(self._on_select_checkitem1_checkbox_stateChanged)
            self.verticalLayout.addWidget(self.checkitem1_checkbox)
        if self.checkitem2:
            self.checkitem2_checkbox = QCheckBox(self.checkitem2)
            self.checkitem2_checkbox.stateChanged.connect(self._on_select_checkitem2_checkbox_stateChanged)
            self.verticalLayout.addWidget(self.checkitem2_checkbox)
        if not items:
            spacerItem = QSpacerItem(1, 1, QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.verticalLayout.addItem(spacerItem)

        self._close_timer = None
        self._closein = closein - 1
        if closein > 0:
            self.closein_label = QLabel("OK in %d sec..." % closein)
            self.closein_label.setAlignment(Qt.AlignRight)
            self.verticalLayout.addWidget(self.closein_label)
            self._close_timer = threading.Timer(1.0, self._on_close_timer)
            self._close_timer.start()

        # create buttons
        self.buttonBox = QDialogButtonBox(self)
        self.buttonBox.setObjectName("buttonBox")
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setStandardButtons(buttons)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        self.verticalLayout.addWidget(self.buttonBox)

        # set the input fields
        if items:
            self.content.createFieldsFromValues(items, exclusive)
            if (select_if_single and len(items) == 1) or preselect_all:
                self.select_all_checkbox.setCheckState(Qt.Checked)

        if not items or len(items) < 7:
            self.filter_field.setVisible(False)

        # restore from configuration file
        self._geometry_name = store_geometry
        if store_geometry and nm.settings().store_geometry:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            settings.beginGroup(store_geometry)
            self.resize(settings.value("size", QSize(480, 320)))
            pos = settings.value("pos", QPoint(0, 0))
            if pos.x() != 0 and pos.y() != 0:
                self.move(pos)
            settings.endGroup()

#    print '=============== create', self.objectName()
#
#  def __del__(self):
#    print "************ destroy", self.objectName()

    def _on_main_toggle(self, state):
        self.cancel_autoclose()
        self._ignore_next_toggle = state != self.select_all_checkbox.checkState()
        self.select_all_checkbox.setCheckState(state)

    def _on_select_all_checkbox_stateChanged(self, state):
        self.cancel_autoclose()
        if not self._ignore_next_toggle:
            self.content.setState(state)
        self._ignore_next_toggle = False

    def _on_select_checkitem1_checkbox_stateChanged(self, state):
        self.cancel_autoclose()
        if state == Qt.Checked:
            self.checkitem1_result = True
        elif state == Qt.Unchecked:
            self.checkitem1_result = False

    def _on_select_checkitem2_checkbox_stateChanged(self, state):
        self.cancel_autoclose()
        if state == Qt.Checked:
            self.checkitem2_result = True
        elif state == Qt.Unchecked:
            self.checkitem2_result = False

    def _on_filter_changed(self):
        self.content.filter(self.filter_field.text())

    def _on_close_timer(self):
        self.closein_label.setText("OK in %d sec..." % self._closein)
        if self._closein == 0:
            self.buttonBox.accepted.emit()
            return
        self._closein -= 1
        self._close_timer = threading.Timer(1.0, self._on_close_timer)
        self._close_timer.start()

    def cancel_autoclose(self):
        if self._close_timer is not None:
            self._close_timer.cancel()
            self._close_timer = None
            self.closein_label.setVisible(False)

    def getSelected(self):
        return self.content.getSelected()

    @staticmethod
    def getValue(title, description='', items=list(), exclusive=False, preselect_all=False, icon='', parent=None, select_if_single=True, checkitem1='', checkitem2='', closein=0, store_geometry=''):
        selectDia = SelectDialog(items, exclusive=exclusive, preselect_all=preselect_all, description=description, icon=icon, parent=parent, select_if_single=select_if_single, checkitem1=checkitem1, checkitem2=checkitem2, closein=closein, store_geometry=store_geometry)
        selectDia.setWindowTitle(title)
        SelectDialog.MODAL_DIALOG = selectDia
        if selectDia.exec_():
            if selectDia.checkitem2:
                return selectDia.getSelected(), True, selectDia.checkitem1_result, selectDia.checkitem2_result
            if selectDia.checkitem1:
                return selectDia.getSelected(), True, selectDia.checkitem1_result
            return selectDia.getSelected(), True
        if selectDia.checkitem2:
            return list(), False, False, False
        if selectDia.checkitem1:
            return list(), False, False
        return list(), False


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%% close handling                        %%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _store_geometry(self):
        if self._geometry_name:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            settings.beginGroup(self._geometry_name)
            settings.setValue("size", self.size())
            settings.setValue("pos", self.pos())
            settings.endGroup()

    def accept(self):
        self._store_geometry()
        self.cancel_autoclose()
        self.setResult(QDialog.Accepted)
        self.hide()

    def reject(self):
        self._store_geometry()
        self.cancel_autoclose()
        self.setResult(QDialog.Rejected)
        self.hide()

    def hideEvent(self, event):
        self.close()

    def closeEvent(self, event):
        '''
        Test the open files for changes and save this if needed.
        '''
        self.cancel_autoclose()
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        SelectDialog.MODAL_DIALOG = None
        QDialog.closeEvent(self, event)


class MainBox(QWidget):
    '''
    A widget with entries.
    '''

    toggled = Signal(Qt.CheckState)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.setObjectName("MainBox")
        self.__on_intern_change = False
        boxLayout = QFormLayout()
        boxLayout.setVerticalSpacing(0)
        self.setLayout(boxLayout)

    def createFieldsFromValues(self, values, exclusive=False):
        self.setUpdatesEnabled(False)
        try:
            if isinstance(values, list):
                for v in values:
                    checkbox = QCheckBox(v)
                    checkbox.toggled.connect(self._on_checkbox_toggled)
                    checkbox.setObjectName(v)
                    checkbox.setAutoExclusive(exclusive)
                    self.layout().addRow(checkbox)
        finally:
            self.setUpdatesEnabled(True)

    def _on_checkbox_toggled(self):
        if not self.__on_intern_change:
            sel_count = self.getSelected()
            if len(sel_count) == 0:
                self.toggled.emit(Qt.Unchecked)
            elif len(sel_count) == self.layout().count():
                self.toggled.emit(Qt.Checked)
            else:
                self.toggled.emit(Qt.PartiallyChecked)

    def filter(self, arg):
        '''
        Hide the parameter input field, which label dosn't contains the C{arg}.
        @param arg: the filter text
        @type arg: C{str}
        '''
        for i in range(self.layout().count()):
            item = self.layout().itemAt(i).widget()
            if isinstance(item, QCheckBox):
                new_state = (not re.search(arg, item.objectName()) is None)
                item.setVisible(new_state)
                if new_state:
                    self._on_checkbox_toggled()

    def getSelected(self):
        result = list()
        for i in range(self.layout().count()):
            item = self.layout().itemAt(i).widget()
            if isinstance(item, QCheckBox):
                if item.isChecked():
                    result.append(item.objectName())
        return result

    def setState(self, state):
        self.__on_intern_change = True
        for i in range(self.layout().count()):
            item = self.layout().itemAt(i).widget()
            if isinstance(item, QCheckBox):
                if state == Qt.Checked:
                    item.setCheckState(Qt.Checked)
                elif state == Qt.Unchecked:
                    item.setCheckState(Qt.Unchecked)
                elif state == Qt.PartiallyChecked and item.isVisible():
                    item.setCheckState(Qt.Checked)
        self.__on_intern_change = False
        self._on_checkbox_toggled()
