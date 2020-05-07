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



from python_qt_binding.QtCore import QObject, Qt, Signal
from python_qt_binding.QtGui import QIcon
import os
import threading

import rospy

from fkie_node_manager_daemon.common import is_package, utf8
from fkie_node_manager_daemon import screen
from fkie_node_manager.detailed_msg_box import MessageBox
from fkie_node_manager.editor.yaml_highlighter import YamlHighlighter

from .editor.text_edit import TextEdit
try:
    from python_qt_binding.QtGui import QApplication, QVBoxLayout, QSizePolicy
    from python_qt_binding.QtGui import QComboBox, QDialog, QDialogButtonBox, QFileDialog, QToolButton
except Exception:
    from python_qt_binding.QtWidgets import QApplication, QVBoxLayout, QSizePolicy
    from python_qt_binding.QtWidgets import QComboBox, QDialog, QDialogButtonBox, QFileDialog, QToolButton
import fkie_node_manager as nm

class SyncHighlighter(YamlHighlighter):
    '''
    Enabled the syntax highlightning for the sync interface.
    '''

    def __init__(self, parent=None):
        YamlHighlighter.__init__(self, parent)
        tagList = ["\\bignore_hosts\\b", "\\bsync_hosts\\b",
                   "\\bignore_nodes\\b", "\\bsync_nodes\\b",
                   "\\bignore_topics\\b", "\\bignore_publishers\\b",
                   "\\bignore_topics\\b", "\\bsync_topics\\b",
                   "\\bignore_subscribers\\b", "\\bsync_services\\b",
                   "\\bsync_topics_on_demand\\b", "\\bsync_remote_nodes\\b",
                   "\\bignore_services\\b", "\\bdo_not_sync\\b",
                   "\\bresync_on_reconnect\\b", "\\bresync_on_reconnect_timeout\\b",
                   ]
        for tag in tagList:
            self.rules.append((self._create_regexp(tag), self._create_format(Qt.darkBlue)))
        self.rules.append((self._create_regexp("\\b\\*|\\*\\B|\\/\\*"), self._create_format(Qt.darkGreen, 'bold')))


class SyncDialog(QDialog):
    '''
    A dialog to set the sync options.
    '''

    def __init__(self, parent=None):
        QDialog.__init__(self, parent)
#    self.host = host
        self.setWindowIcon(nm.settings().icon('irondevil_sync.png'))
        self.setWindowTitle('Sync')
        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")
        self.resize(350, 190)

        self.toolButton_SyncAll = QToolButton(self)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(2)
        sizePolicy.setHeightForWidth(self.toolButton_SyncAll.sizePolicy().hasHeightForWidth())
        self.toolButton_SyncAll.setSizePolicy(sizePolicy)
        self.toolButton_SyncAll.setObjectName("toolButton_SyncAll")
        self.verticalLayout.addWidget(self.toolButton_SyncAll)
        self.toolButton_SyncAll.setText(self._translate("Sync All"))
        self.toolButton_SyncAll.clicked.connect(self._on_sync_all_clicked)

#     self.toolButton_SyncAllAnyMsg = QToolButton(self)
#     sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
#     sizePolicy.setHorizontalStretch(0)
#     sizePolicy.setVerticalStretch(1)
#     sizePolicy.setHeightForWidth(self.toolButton_SyncAllAnyMsg.sizePolicy().hasHeightForWidth())
#     self.toolButton_SyncAllAnyMsg.setSizePolicy(sizePolicy)
#     self.toolButton_SyncAllAnyMsg.setObjectName("toolButton_SyncAllAnyMsg")
#     self.verticalLayout.addWidget(self.toolButton_SyncAllAnyMsg)
#     self.toolButton_SyncAllAnyMsg.setText(self._translate("Sync all (+AnyMsg)"))
#     self.toolButton_SyncAllAnyMsg.clicked.connect(self._on_sync_all_anymsg_clicked)

        self.toolButton_SyncTopicOnDemand = QToolButton(self)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.toolButton_SyncTopicOnDemand.sizePolicy().hasHeightForWidth())
        self.toolButton_SyncTopicOnDemand.setSizePolicy(sizePolicy)
        self.toolButton_SyncTopicOnDemand.setObjectName("toolButton_SyncTopicOnDemand")
        self.verticalLayout.addWidget(self.toolButton_SyncTopicOnDemand)
        self.toolButton_SyncTopicOnDemand.setText(self._translate("Sync only topics on demand"))
        self.toolButton_SyncTopicOnDemand.clicked.connect(self._on_sync_topics_on_demand_clicked)

        self.toolButton_SelectInterface = QToolButton(self)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.toolButton_SelectInterface.sizePolicy().hasHeightForWidth())
        self.toolButton_SelectInterface.setSizePolicy(sizePolicy)
        self.toolButton_SelectInterface.setObjectName("toolButton_SelectInterface")
        self.verticalLayout.addWidget(self.toolButton_SelectInterface)
        self.toolButton_SelectInterface.setText(self._translate("Select an interface"))
        self.toolButton_SelectInterface.clicked.connect(self._on_select_interface_clicked)

        self.interface_field = QComboBox(self)
        self.interface_field.setInsertPolicy(QComboBox.InsertAlphabetically)
        self.interface_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.interface_field.setEditable(True)
        self.interface_field.setVisible(False)
        self.interface_field.setObjectName("interface_field")
        self.verticalLayout.addWidget(self.interface_field)
        self.interface_field.currentIndexChanged[str].connect(self._on_interface_selected)

        self.toolButton_EditInterface = QToolButton(self)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.toolButton_EditInterface.sizePolicy().hasHeightForWidth())
        self.toolButton_EditInterface.setSizePolicy(sizePolicy)
        self.toolButton_EditInterface.setObjectName("toolButton_EditInterface")
        self.verticalLayout.addWidget(self.toolButton_EditInterface)
        self.toolButton_EditInterface.setText(self._translate("Edit selected interface"))
        self.toolButton_EditInterface.clicked.connect(self._on_edit_interface_clicked)
        self.toolButton_EditInterface.setVisible(False)

        self.toolButton_CreateInterface = QToolButton(self)
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.toolButton_CreateInterface.sizePolicy().hasHeightForWidth())
        self.toolButton_CreateInterface.setSizePolicy(sizePolicy)
        self.toolButton_CreateInterface.setObjectName("toolButton_CreateInterface")
        self.verticalLayout.addWidget(self.toolButton_CreateInterface)
        self.toolButton_CreateInterface.setText(self._translate("Create an interface"))
        self.toolButton_CreateInterface.clicked.connect(self._on_create_interface_clicked)
        self.toolButton_CreateInterface.setVisible(False)

        self.textedit = TextEdit('', self)
        self.hl = SyncHighlighter(self.textedit.document())
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.textedit.setSizePolicy(sizePolicy)
        self.textedit.setObjectName("syncedit")
        self.verticalLayout.addWidget(self.textedit)
        self.textedit.setVisible(False)

        self._fill_interface_thread = None
        self._interfaces_files = None
        self._sync_args = []
        self._interface_filename = None

        self.buttonBox = QDialogButtonBox(self)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel)
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        self._new_iface = True

    def _translate(self, text):
        if hasattr(QApplication, "UnicodeUTF8"):
            return QApplication.translate("Form", text, None, QApplication.UnicodeUTF8)
        else:
            return QApplication.translate("Form", text, None)

    @property
    def sync_args(self):
        return self._sync_args

    @property
    def interface_filename(self):
        return self._interface_filename

    def _on_sync_all_clicked(self):
        self.setResult(QDialog.Accepted)
        self._sync_args = []
        self._sync_args.append(''.join(['_interface_url:=', "'.'"]))
        self._sync_args.append(''.join(['_sync_topics_on_demand:=', 'False']))
        self._sync_args.append(''.join(['_ignore_hosts:=', '[]']))
        self._sync_args.append(''.join(['_sync_hosts:=', '[]']))
        self._sync_args.append(''.join(['_ignore_nodes:=', '[]']))
        self._sync_args.append(''.join(['_sync_nodes:=', '[]']))
        self._sync_args.append(''.join(['_ignore_topics:=', '[]']))
        self._sync_args.append(''.join(['_ignore_publishers:=', '[]']))
        self._sync_args.append(''.join(['_ignore_subscribers:=', '[]']))
        self._sync_args.append(''.join(['_sync_topics:=', '[]']))
        self._sync_args.append(''.join(['_ignore_services:=', '[]']))
        self._sync_args.append(''.join(['_sync_services:=', '[]']))
        self._sync_args.append(''.join(['_sync_remote_nodes:=', 'False']))
        self._interface_filename = None
        self.accept()

#   def _on_sync_all_anymsg_clicked(self):
#     self._sync_args = []
#     self._sync_args.append(''.join(['_interface_url:=', "'.'"]))
#     self._sync_args.append(''.join(['_sync_topics_on_demand:=', 'True']))
#     self._sync_args.append(''.join(['_ignore_hosts:=', '[]']))
#     self._sync_args.append(''.join(['_sync_hosts:=', '[]']))
#     self._sync_args.append(''.join(['_ignore_nodes:=', '[]']))
#     self._sync_args.append(''.join(['_sync_nodes:=', '[]']))
#     self._sync_args.append(''.join(['_ignore_topics:=', '[]']))
#     self._sync_args.append(''.join(['_sync_topics:=', '[/*]']))
#     self._sync_args.append(''.join(['_ignore_services:=', '[]']))
#     self._sync_args.append(''.join(['_sync_services:=', '[]']))
#     self._sync_args.append(''.join(['_sync_remote_nodes:=', 'False']))
#     self._interface_filename = None
#     self.accept()

    def _on_sync_topics_on_demand_clicked(self):
        self._sync_args = []
        self._sync_args.append(''.join(['_interface_url:=', "'.'"]))
        self._sync_args.append(''.join(['_sync_topics_on_demand:=', 'True']))
        self._sync_args.append(''.join(['_ignore_hosts:=', '[]']))
        self._sync_args.append(''.join(['_sync_hosts:=', '[]']))
        self._sync_args.append(''.join(['_ignore_nodes:=', '[]']))
        self._sync_args.append(''.join(['_sync_nodes:=', '[]']))
        self._sync_args.append(''.join(['_ignore_topics:=', '[]']))
        self._sync_args.append(''.join(['_ignore_publishers:=', '[]']))
        self._sync_args.append(''.join(['_ignore_subscribers:=', '[]']))
        self._sync_args.append(''.join(['_sync_topics:=', '[/only_on_demand]']))
        self._sync_args.append(''.join(['_ignore_services:=', '[/*]']))
        self._sync_args.append(''.join(['_sync_services:=', '[]']))
        self._sync_args.append(''.join(['_sync_remote_nodes:=', 'False']))
        self._interface_filename = None
        self.accept()

    def _on_select_interface_clicked(self):
        self.toolButton_SyncAll.setVisible(False)
#    self.toolButton_SyncAllAnyMsg.setVisible(False)
        self.toolButton_SyncTopicOnDemand.setVisible(False)
        self.toolButton_SelectInterface.setVisible(False)
        self.interface_field.setVisible(True)
        self.toolButton_CreateInterface.setVisible(True)
        self.toolButton_EditInterface.setVisible(True)
        self.toolButton_EditInterface.setEnabled(False)
        self.textedit.setVisible(False)
#    # fill the interfaces
        if self._interfaces_files is None:
            self.interface_field.addItems(['interface searching...'])
            self.interface_field.setCurrentIndex(0)
            self._fill_interface_thread = InterfacesThread()
            self._fill_interface_thread.interfaces.connect(self._fill_interfaces)
            self._fill_interface_thread.start()
        else:
            self.toolButton_EditInterface.setEnabled(self.interface_field.currentText() in self._interfaces_files)
        self.buttonBox.clear()
        self.buttonBox.setStandardButtons(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.interface_field.setFocus(Qt.TabFocusReason)
        self.resize(350, 80)

    def _fill_interfaces(self, interfaces_files):
        self._interfaces_files = interfaces_files
        self.interface_field.clear()
        self.interface_field.clearEditText()
        self.interface_field.addItems(list(self._interfaces_files.keys()))

    def _on_interface_selected(self, interface):
        if self._interfaces_files and interface in self._interfaces_files:
            self._sync_args = []
            self._sync_args.append(''.join(['_interface_url:=', interface]))
            self.toolButton_EditInterface.setEnabled(True)
        else:
            self.toolButton_EditInterface.setEnabled(False)

    def accept(self):
        if self.textedit.isVisible():
            try:
                tmp_file = os.path.join(screen.LOG_PATH, 'tmp_sync_interface.sync')
                with open(tmp_file, 'w+') as f:
                    f.write(self.textedit.toPlainText())
                from fkie_master_discovery.common import read_interface
                read_interface(tmp_file)
                if not self._new_iface and self.interface_field.currentText() in self._interfaces_files:
                    fileName = self._interfaces_files[self.interface_field.currentText()]
                else:
                    fileName, _ = QFileDialog.getSaveFileName(self, 'Save sync interface', '/home', "Sync Files (*.sync)")
                if fileName:
                    with open(fileName, 'w+') as f:
                        self._interface_filename = fileName
                        f.write(self.textedit.toPlainText())
                        if self._new_iface:
                            self.interface_field.clear()
                            self._interfaces_files = None
                        self._on_select_interface_clicked()
#        QDialog.accept(self)
#        self.resetView()
            except Exception as e:
                MessageBox.warning(self, "Create sync interface",
                                   "Error while create interface",
                                   utf8(e))
        elif self.interface_field.isVisible():
            interface = self.interface_field.currentText()
            if self._interfaces_files and interface in self._interfaces_files:
                self._interface_filename = self._interfaces_files[interface]
                self._sync_args = []
                self._sync_args.append(''.join(['_interface_url:=', interface]))
                QDialog.accept(self)
                self.resetView()
        else:
            QDialog.accept(self)
            self.resetView()

    def reject(self):
        if self.textedit.isVisible():
            self._on_select_interface_clicked()
        else:
            QDialog.reject(self)
            self.resetView()

    def _on_create_interface_clicked(self):
        self._new_iface = True
        self.interface_field.setVisible(False)
        self.toolButton_CreateInterface.setVisible(False)
        self.toolButton_EditInterface.setVisible(False)
        self.textedit.setVisible(True)
        self.textedit.setText("# The ignore_* lists will be processed first.\n"
                              "# For ignore/sync nodes, topics or services\n"
                              "# use follow declaration:\n"
                              "#{param name}: \n"
                              "#   - {ros name}\n"
                              "# or for selected hosts:\n"
                              "#   - {host name}:\n"
                              "#     - {ros name}\n\n"
                              "# you can use follow wildcard: '*', but not as a first character\n"
                              "ignore_hosts:\n"
                              "sync_hosts:\n\n"
                              "ignore_nodes:\n"
                              "sync_nodes:\n\n"
                              "ignore_topics:\n"
                              "ignore_publishers:\n"
                              "ignore_subscribers:\n"
                              "sync_topics:\n\n"
                              "ignore_services:\n"
                              "  - /*get_loggers\n"
                              "  - /*set_logger_level\n"
                              "sync_services:\n\n"
                              "# If sync_topics_on_demand is True the local subscribed and published topics\n"
                              "# are synchronized with remote even if they are not in the sync_* list.\n"
                              "sync_topics_on_demand: False\n\n"
                              "# The nodes which are running not at the same host as the ROS master are not\n"
                              "# synchronized by default. Use sync_remote_nodes to sync these nodes also.\n"
                              "sync_remote_nodes: False\n\n"
                              )
        self.resize(350, 300)

    def _on_edit_interface_clicked(self):
        self._new_iface = False
        self.interface_field.setVisible(False)
        self.toolButton_CreateInterface.setVisible(False)
        self.toolButton_EditInterface.setVisible(False)
        self.textedit.setVisible(True)
        if self.interface_field.currentText() in self._interfaces_files:
            try:
                with open(self._interfaces_files[self.interface_field.currentText()], 'rw') as f:
                    iface = f.read()
                    self.textedit.setText(iface)
            except Exception as e:
                MessageBox.warning(self, "Edit sync interface",
                                   "Error while open interface",
                                   utf8(e))
        self.resize(350, 300)

    def resetView(self):
        self.toolButton_SyncAll.setVisible(True)
#     self.toolButton_SyncAllAnyMsg.setVisible(True)
        self.toolButton_SyncTopicOnDemand.setVisible(True)
        self.toolButton_SelectInterface.setVisible(True)
        self.interface_field.setVisible(False)
        self.toolButton_CreateInterface.setVisible(False)
        self.toolButton_EditInterface.setVisible(False)
        self.textedit.setVisible(False)
        self.buttonBox.clear()
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel)
        self.resize(350, 160)


class InterfacesThread(QObject, threading.Thread):
    '''
    A thread to list all sync interface files and
    publish there be sending a QT signal.
    '''
    interfaces = Signal(dict)
    '''
  :ivar: interfaces is a signal, which is emitted, if a list with sync files was created.
  '''

    def __init__(self):
        '''
        '''
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._interfaces_files = None
        self.setDaemon(True)

    def run(self):
        '''
        '''
        try:
            # fill the input fields
            self.root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
            self._interfaces_files = {}
            for p in self.root_paths:
                ret = self._get_interfaces(p)
                self._interfaces_files.update(ret)
            self.interfaces.emit(self._interfaces_files)
        except Exception:
            import traceback
            formatted_lines = traceback.format_exc(1).splitlines()
            rospy.logwarn("Error while list sync interfaces:\n\t%s", formatted_lines[-1])

    def _get_interfaces(self, path, package=None):
        # TODO: get interfaces from nmd
        result = {}
        if os.path.isdir(path):
            fileList = os.listdir(path)
            # set package, if it is currently None and one found
            if not package:
                # detect package
                if is_package(fileList):
                    package = os.path.basename(path)
            for f in fileList:
                ret = self._get_interfaces(os.path.join(path, f), package)
                result.update(ret)
        elif package and os.path.isfile(path) and path.endswith('.sync'):
            # create a selection for binaries
            return {'pkg://%s///%s' % (package, os.path.basename(path)): path}
        return result
