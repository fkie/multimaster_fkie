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



from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QImage, QPixmap

try:
    from python_qt_binding.QtGui import QCheckBox, QPushButton, QSpacerItem, QSizePolicy, QTextEdit, QDialog
    from python_qt_binding.QtGui import QDialogButtonBox, QVBoxLayout, QHBoxLayout, QLabel, QStyle, QApplication
except Exception:
    from python_qt_binding.QtWidgets import QCheckBox, QPushButton, QSpacerItem, QSizePolicy, QTextEdit, QDialog
    from python_qt_binding.QtWidgets import QDialogButtonBox, QVBoxLayout, QHBoxLayout, QLabel, QStyle, QApplication
import fkie_node_manager as nm

IGNORED_ERRORS = []


class DetailedError(Exception):
    ''' '''

    def __init__(self, title, text, detailed_text=""):
        self.title = title
        self.text = text
        self.detailed_text = detailed_text

    def __str__(self):
        return repr(self.text) + "::" + self.detailed_text


class MessageBox(QDialog):

    NoIcon = 0
    Information = 1
    Warning = 2
    Critical = 3
    Question = 4

    NoButton = 0
    Ok = 1         # An "OK" button defined with the AcceptRole .
    Open = 2       # A "Open" button defined with the AcceptRole .
    Save = 4       # A "Save" button defined with the AcceptRole .
    Cancel = 8     # A "Cancel" button defined with the RejectRole .
    Close = 16     # A "Close" button defined with the RejectRole .
    Discard = 32   # A "Discard" or "Don't Save" button, depending on the platform, defined with the DestructiveRole .
    Apply = 64     # An "Apply" button defined with the ApplyRole .
    Reset = 128    # A "Reset" button defined with the ResetRole .
    RestoreDefaults = 256  # A "Restore Defaults" button defined with the ResetRole .
    Help = 512       # A "Help" button defined with the HelpRole .
    SaveAll = 1024   # A "Save All" button defined with the AcceptRole .
    Yes = 2048       # A "Yes" button defined with the YesRole .
    YesToAll = 4096  # A "Yes to All" button defined with the YesRole .
    No = 8192        # A "No" button defined with the NoRole .
    NoToAll = 16384  # A "No to All" button defined with the NoRole .
    Abort = 32768    # An "Abort" button defined with the RejectRole .
    Retry = 65536    # A "Retry" button defined with the AcceptRole .
    Ignore = 131072  # An "Ignore" button defined with the AcceptRole .
    Avoid = 262144   # An "'Don't show again'" button defined with the HelpRole, returns a default AcceptButton .

    def __init__(self, icon, title, text, detailed_text="", buttons=Cancel | Ok, parent=None):
        QDialog.__init__(self, parent=parent)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowTitleHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowContextHelpButtonHint & ~Qt.WindowMinimizeButtonHint)
        self.setObjectName('MessageBox')
        self._use_checkbox = True
        self.text = text
        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(1, 1, 1, 1)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.horizontalLayout.setContentsMargins(1, 1, 1, 1)
        # create icon
        pixmap = None
        if icon == self.NoIcon:
            pass
        elif icon == self.Question:
            pixmap = nm.settings().pixmap('question.png').scaled(56, 56, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        elif icon == self.Information:
            pixmap = nm.settings().pixmap('info.png').scaled(56, 56, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        elif icon == self.Warning:
            pixmap = nm.settings().pixmap('warning.png').scaled(56, 56, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        elif icon == self.Critical:
            pixmap = nm.settings().pixmap('critical.png').scaled(56, 56, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
        spacerItem = QSpacerItem(10, 60, QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.icon_label = QLabel()
        if pixmap is not None:
            self.icon_label.setPixmap(pixmap)
        self.icon_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.horizontalLayout.addWidget(self.icon_label)
        spacerItem = QSpacerItem(10, 60, QSizePolicy.Minimum, QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        # add message
        self.message_label = QLabel(text)
        self.message_label.setWordWrap(True)
        self.message_label.setScaledContents(True)
        self.message_label.setOpenExternalLinks(True)
        self.horizontalLayout.addWidget(self.message_label)
        self.verticalLayout.addLayout(self.horizontalLayout)

        # create buttons
        self.buttonBox = QDialogButtonBox(self)
        self.buttonBox.setObjectName("buttonBox")
        self.buttonBox.setOrientation(Qt.Horizontal)

        self._accept_button = None
        self._reject_button = None
        self._buttons = buttons
        self._create_buttons(buttons)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        self.verticalLayout.addWidget(self.buttonBox)

        if detailed_text:
            self.btn_show_details = QPushButton(self.tr('Details...'))
            self.btn_show_details.setCheckable(True)
            self.btn_show_details.setChecked(True)
            self.btn_show_details.toggled.connect(self.on_toggled_details)
            self.buttonBox.addButton(self.btn_show_details, QDialogButtonBox.ActionRole)
            # create area for detailed text
            self.textEdit = textEdit = QTextEdit(self)
            textEdit.setObjectName("textEdit")
            textEdit.setReadOnly(True)
            textEdit.setText(detailed_text)
            # textEdit.setVisible(False)
            self.verticalLayout.addWidget(self.textEdit)
        self.resize(480, self.verticalLayout.totalSizeHint().height())
        buttons_in_box = self.buttonBox.buttons()
        if buttons_in_box:
            self.buttonBox.buttons()[0].setFocus()

    def setAcceptButton(self, button):
        '''
        Sets the button with given ID to accept button if more then one button with AcceptRole was added to this dialog.
        Adds the buttton to the box if is not already in.
        :param int button: button id
        '''
        if not button & self._buttons:
            self._create_buttons(button)
        self._accept_button = button

    def setRejectButton(self, button):
        '''
        Sets the button with given ID to reject button if more then one button with RejectRole was added to this dialog.
        Adds the buttton to the box if is not already in.
        :param int button: button id
        '''
        if not button & self._buttons:
            self._create_buttons(button)
        self._reject_button = button

    def on_toggled_details(self, checked):
        if checked:
            self.verticalLayout.addWidget(self.textEdit)
        else:
            self.verticalLayout.removeWidget(self.textEdit)
        self.textEdit.setVisible(checked)
        if not self.isMaximized():
            self.setMinimumSize(self.verticalLayout.totalMinimumSize())
            self.resize(self._current_size.width(), self.verticalLayout.totalSizeHint().height())

    @staticmethod
    def about(parent, title, text, detailed_text='', buttons=Close):
        box = MessageBox(MessageBox.Information, title, text, detailed_text=detailed_text, buttons=buttons, parent=parent)
        if MessageBox.Yes & buttons:
            box.setAcceptButton(MessageBox.Yes)
        if MessageBox.Cancel & buttons:
            box.setRejectButton(MessageBox.Cancel)
        elif MessageBox.No & buttons:
            box.setRejectButton(MessageBox.No)
        return box.exec_()

    @staticmethod
    def information(parent, title, text, detailed_text='', buttons=Close):
        box = MessageBox(MessageBox.Information, title, text, detailed_text=detailed_text, buttons=buttons, parent=parent)
        if MessageBox.Yes & buttons:
            box.setAcceptButton(MessageBox.Yes)
        if MessageBox.Cancel & buttons:
            box.setRejectButton(MessageBox.Cancel)
        elif MessageBox.No & buttons:
            box.setRejectButton(MessageBox.No)
        return box.exec_()

    @staticmethod
    def question(parent, title, text, detailed_text='', buttons=Yes | No | Cancel):
        box = MessageBox(MessageBox.Question, title, text, detailed_text=detailed_text, buttons=buttons, parent=parent)
        if MessageBox.Yes & buttons:
            box.setAcceptButton(MessageBox.Yes)
        if MessageBox.Cancel & buttons:
            box.setRejectButton(MessageBox.Cancel)
        elif MessageBox.No & buttons:
            box.setRejectButton(MessageBox.No)
        return box.exec_()

    @staticmethod
    def warning(parent, title, text, detailed_text='', buttons=Ok):
        box = MessageBox(MessageBox.Warning, title, text, detailed_text=detailed_text, buttons=buttons, parent=parent)
        if MessageBox.Yes & buttons:
            box.setAcceptButton(MessageBox.Yes)
        if MessageBox.Cancel & buttons:
            box.setRejectButton(MessageBox.Cancel)
        elif MessageBox.No & buttons:
            box.setRejectButton(MessageBox.No)
        return box.exec_()

    @staticmethod
    def critical(parent, title, text, detailed_text='', buttons=Ok):
        box = MessageBox(MessageBox.Critical, title, text, detailed_text=detailed_text, buttons=buttons, parent=parent)
        if MessageBox.Yes & buttons:
            box.setAcceptButton(MessageBox.Yes)
        if MessageBox.Cancel & buttons:
            box.setRejectButton(MessageBox.Cancel)
        elif MessageBox.No & buttons:
            box.setRejectButton(MessageBox.No)
        return box.exec_()

    def resizeEvent(self, event):
        if not self.isMaximized():
            self._current_size = event.size()

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%% close handling                        %%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def exec_(self):
        if self.text in IGNORED_ERRORS:
            self.accept()
            return self.result()
        return QDialog.exec_(self)

    def accept(self):
        if self.result() == 0:
            if self._accept_button is not None:
                self.setResult(self._accept_button)
            else:
                self.setResult(1)
        self.accepted.emit()
        if self.isModal():
            self.hide()

    def reject(self):
        if self.result() == 0:
            if self._reject_button is not None:
                self.setResult(self._reject_button)
        self.rejected.emit()
        self.hide()

    def hideEvent(self, event):
        # event.ignore()
        self.close()

    def closeEvent(self, event):
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        QDialog.closeEvent(self, event)

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%% create buttons                        %%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _create_buttons(self, buttons):
        if MessageBox.Ok & buttons:
            self._accept_button = MessageBox.Ok
            bt = QPushButton(self.tr("&ok"))
            bt.clicked.connect(self._on_ok_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.AcceptRole)
        if MessageBox.Open & buttons:
            self._accept_button = MessageBox.Open
            bt = QPushButton(self.tr("&Open"))
            bt.clicked.connect(self._on_open_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.AcceptRole)
        if MessageBox.Save & buttons:
            self._accept_button = MessageBox.Save
            bt = QPushButton(self.tr("&Save"))
            bt.clicked.connect(self._on_save_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.AcceptRole)
        if MessageBox.Cancel & buttons:
            self._reject_button = MessageBox.Cancel
            bt = QPushButton(self.tr("&Cancel"))
            bt.clicked.connect(self._on_cancel_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.RejectRole)
        if MessageBox.Close & buttons:
            self._reject_button = MessageBox.Close
            bt = QPushButton(self.tr("&Close"))
            bt.clicked.connect(self._on_close_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.RejectRole)
        if MessageBox.Discard & buttons:
            bt = QPushButton(self.tr("&Discard"))
            bt.clicked.connect(self._on_discard_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.DestructiveRole)
        if MessageBox.Apply & buttons:
            bt = QPushButton(self.tr("&Apply"))
            bt.clicked.connect(self._on_apply_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.ApplyRole)
        if MessageBox.Reset & buttons:
            bt = QPushButton(self.tr("&Reset"))
            bt.clicked.connect(self._on_reset_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.ResetRole)
        if MessageBox.RestoreDefaults & buttons:
            bt = QPushButton(self.tr("&RestoreDefaults"))
            bt.clicked.connect(self._on_restore_defaults_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.ResetRole)
        if MessageBox.Help & buttons:
            bt = QPushButton(self.tr("&Help"))
            bt.clicked.connect(self._on_help_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.HelpRole)
        if MessageBox.SaveAll & buttons:
            self._accept_button = MessageBox.SaveAll
            bt = QPushButton(self.tr("&SaveAll"))
            bt.clicked.connect(self._on_saveall_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.AcceptRole)
        if MessageBox.Yes & buttons:
            bt = QPushButton(self.tr("&Yes"))
            bt.clicked.connect(self._on_yes_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.YesRole)
        if MessageBox.YesToAll & buttons:
            bt = QPushButton(self.tr("&YesToAll"))
            bt.clicked.connect(self._on_yestoall_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.YesRole)
        if MessageBox.No & buttons:
            bt = QPushButton(self.tr("&No"))
            bt.clicked.connect(self._on_no_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.NoRole)
        if MessageBox.NoToAll & buttons:
            bt = QPushButton(self.tr("&NoToAll"))
            bt.clicked.connect(self._on_notoall_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.NoRole)
        if MessageBox.Abort & buttons:
            self._reject_button = MessageBox.Abort
            bt = QPushButton(self.tr("&Abort"))
            bt.clicked.connect(self._on_abort_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.RejectRole)
        if MessageBox.Retry & buttons:
            self._accept_button = MessageBox.Retry
            bt = QPushButton(self.tr("&Retry"))
            bt.clicked.connect(self._on_retry_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.AcceptRole)
        if MessageBox.Ignore & buttons:
            self._accept_button = MessageBox.Ignore
            bt = QPushButton(self.tr("&Ignore"))
            bt.clicked.connect(self._on_ignore_clicked)
            self.buttonBox.addButton(bt, QDialogButtonBox.AcceptRole)
        if MessageBox.Avoid & buttons:
            if self._use_checkbox:
                checkbox = QCheckBox("&Don't show again", self)
                checkbox.stateChanged.connect(self._check_ignore)
                self.buttonBox.addButton(checkbox, QDialogButtonBox.HelpRole)
            else:
                bt = QPushButton(self.tr("&Don't show again"))
                bt.setMaximumHeight(24)
                bt.clicked.connect(self._add_to_ignore)
                self.buttonBox.addButton(bt, QDialogButtonBox.HelpRole)

    def _on_ok_clicked(self):
        self.done(MessageBox.Ok)

    def _on_open_clicked(self):
        self.done(MessageBox.Open)

    def _on_save_clicked(self):
        self.done(MessageBox.Save)

    def _on_cancel_clicked(self):
        self.done(MessageBox.Cancel)

    def _on_close_clicked(self):
        self.done(MessageBox.Close)

    def _on_discard_clicked(self):
        self.done(MessageBox.Discard)

    def _on_apply_clicked(self):
        self.done(MessageBox.Apply)

    def _on_reset_clicked(self):
        self.done(MessageBox.Reset)

    def _on_restore_defaults_clicked(self):
        self.done(MessageBox.RestoreDefaults)

    def _on_help_clicked(self):
        self.done(MessageBox.Help)

    def _on_saveall_clicked(self):
        self.done(MessageBox.SaveAll)

    def _on_yes_clicked(self):
        self.done(MessageBox.Yes)

    def _on_yestoall_clicked(self):
        self.done(MessageBox.YesToAll)

    def _on_no_clicked(self):
        self.done(MessageBox.No)

    def _on_notoall_clicked(self):
        self.done(MessageBox.NoToAll)

    def _on_abort_clicked(self):
        self.done(MessageBox.Abort)

    def _on_retry_clicked(self):
        self.done(MessageBox.Retry)

    def _on_ignore_clicked(self):
        self.done(MessageBox.Ignore)

    def _add_to_ignore(self):
        IGNORED_ERRORS.append(self.text)
        self.accept()

    def _check_ignore(self, state):
        if state:
            IGNORED_ERRORS.append(self.text)
        else:
            try:
                IGNORED_ERRORS.remove(self.text)
            except Exception:
                pass
