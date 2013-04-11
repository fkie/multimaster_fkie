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

from PySide import QtGui
from PySide import QtCore

import os
import threading

import node_manager_fkie as nm
from detailed_msg_box import WarningMessageBox


class SyncDialog(QtGui.QDialog):
  '''
  A dialog to set the sync options.
  '''

  def __init__(self, parent=None):
    QtGui.QDialog.__init__(self, parent)
#    self.host = host
    self.setWindowTitle('Sync')
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.resize(350,160)

    self.toolButton_SyncAll = QtGui.QToolButton(self)
    sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(2)
    sizePolicy.setHeightForWidth(self.toolButton_SyncAll.sizePolicy().hasHeightForWidth())
    self.toolButton_SyncAll.setSizePolicy(sizePolicy)
    self.toolButton_SyncAll.setObjectName("toolButton_SyncAll")
    self.verticalLayout.addWidget(self.toolButton_SyncAll)
    self.toolButton_SyncAll.setText(QtGui.QApplication.translate("Form", "Sync All", None, QtGui.QApplication.UnicodeUTF8))
    self.toolButton_SyncAll.clicked.connect(self._on_sync_all_clicked)

    self.toolButton_SyncTopicOnDemand = QtGui.QToolButton(self)
    sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(1)
    sizePolicy.setHeightForWidth(self.toolButton_SyncTopicOnDemand.sizePolicy().hasHeightForWidth())
    self.toolButton_SyncTopicOnDemand.setSizePolicy(sizePolicy)
    self.toolButton_SyncTopicOnDemand.setObjectName("toolButton_SyncTopicOnDemand")
    self.verticalLayout.addWidget(self.toolButton_SyncTopicOnDemand)
    self.toolButton_SyncTopicOnDemand.setText(QtGui.QApplication.translate("Form", "Sync only topics on demand", None, QtGui.QApplication.UnicodeUTF8))
    self.toolButton_SyncTopicOnDemand.clicked.connect(self._on_sync_topics_on_demand_clicked)

    self.toolButton_SelectInterface = QtGui.QToolButton(self)
    sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(1)
    sizePolicy.setHeightForWidth(self.toolButton_SelectInterface.sizePolicy().hasHeightForWidth())
    self.toolButton_SelectInterface.setSizePolicy(sizePolicy)
    self.toolButton_SelectInterface.setObjectName("toolButton_SelectInterface")
    self.verticalLayout.addWidget(self.toolButton_SelectInterface)
    self.toolButton_SelectInterface.setText(QtGui.QApplication.translate("Form", "Select an interface", None, QtGui.QApplication.UnicodeUTF8))
    self.toolButton_SelectInterface.clicked.connect(self._on_select_interface_clicked)

    self.interface_field = QtGui.QComboBox(self)
    self.interface_field.setInsertPolicy(QtGui.QComboBox.InsertAlphabetically)
    self.interface_field.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed))
    self.interface_field.setEditable(True)
    self.interface_field.setVisible(False)
    self.interface_field.setObjectName("interface_field")
    self.verticalLayout.addWidget(self.interface_field)
    self.interface_field.currentIndexChanged[str].connect(self._on_interface_selected)

    self.toolButton_EditInterface = QtGui.QToolButton(self)
    sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(1)
    sizePolicy.setHeightForWidth(self.toolButton_EditInterface.sizePolicy().hasHeightForWidth())
    self.toolButton_EditInterface.setSizePolicy(sizePolicy)
    self.toolButton_EditInterface.setObjectName("toolButton_EditInterface")
    self.verticalLayout.addWidget(self.toolButton_EditInterface)
    self.toolButton_EditInterface.setText(QtGui.QApplication.translate("Form", "Edit selected interface", None, QtGui.QApplication.UnicodeUTF8))
    self.toolButton_EditInterface.clicked.connect(self._on_edit_interface_clicked)
    self.toolButton_EditInterface.setVisible(False)

    self.toolButton_CreateInterface = QtGui.QToolButton(self)
    sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(1)
    sizePolicy.setHeightForWidth(self.toolButton_CreateInterface.sizePolicy().hasHeightForWidth())
    self.toolButton_CreateInterface.setSizePolicy(sizePolicy)
    self.toolButton_CreateInterface.setObjectName("toolButton_CreateInterface")
    self.verticalLayout.addWidget(self.toolButton_CreateInterface)
    self.toolButton_CreateInterface.setText(QtGui.QApplication.translate("Form", "Create an interface", None, QtGui.QApplication.UnicodeUTF8))
    self.toolButton_CreateInterface.clicked.connect(self._on_create_interface_clicked)
    self.toolButton_CreateInterface.setVisible(False)

    self.textedit = QtGui.QTextEdit(self)
    sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
    self.textedit.setSizePolicy(sizePolicy)
    self.textedit.setObjectName("textedit")
    self.verticalLayout.addWidget(self.textedit)
    self.textedit.setVisible(False)


    self._fill_interface_thread = None
    self._interfaces_files = None
    self._sync_args = []
    
    self.buttonBox = QtGui.QDialogButtonBox(self)
    self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel)
    self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
    self.buttonBox.setObjectName("buttonBox")
    self.verticalLayout.addWidget(self.buttonBox)
    QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), self.accept)
    QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), self.reject)

    self._new_iface = True
  
  @property
  def sync_args(self):
    return self._sync_args
  
  def _on_sync_all_clicked(self):
    self.setResult(QtGui.QDialog.Accepted)
    self._sync_args = []
    self._sync_args.append(''.join(['_interface_url:=', "'.'"]))
    self._sync_args.append(''.join(['_sync_topics_on_demand:=', 'False']))
    self._sync_args.append(''.join(['_ignore_hosts:=', '[]']))
    self._sync_args.append(''.join(['_sync_hosts:=', '[]']))
    self._sync_args.append(''.join(['_ignore_nodes:=', '[]']))
    self._sync_args.append(''.join(['_sync_nodes:=', '[]']))
    self._sync_args.append(''.join(['_ignore_topics:=', '[]']))
    self._sync_args.append(''.join(['_sync_topics:=', '[]']))
    self._sync_args.append(''.join(['_ignore_services:=', '[]']))
    self._sync_args.append(''.join(['_sync_services:=', '[]']))
    self.accept()

  def _on_sync_topics_on_demand_clicked(self):
    self._sync_args = []
    self._sync_args.append(''.join(['_interface_url:=', "'.'"]))
    self._sync_args.append(''.join(['_sync_topics_on_demand:=', 'True']))
    self._sync_args.append(''.join(['_ignore_hosts:=', '[]']))
    self._sync_args.append(''.join(['_sync_hosts:=', '[]']))
    self._sync_args.append(''.join(['_ignore_nodes:=', '[]']))
    self._sync_args.append(''.join(['_sync_nodes:=', '[]']))
    self._sync_args.append(''.join(['_ignore_topics:=', '[]']))
    self._sync_args.append(''.join(['_sync_topics:=', '[/only_on_demand]']))
    self._sync_args.append(''.join(['_ignore_services:=', '[/*]']))
    self._sync_args.append(''.join(['_sync_services:=', '[]']))
    self.accept()

  def _on_select_interface_clicked(self):
    self.toolButton_SyncAll.setVisible(False)
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
      self._fill_interface_thread = threading.Thread(target=self._fill_interfaces)
      self._fill_interface_thread.start()
    else:
      self.toolButton_EditInterface.setEnabled(self._interfaces_files.has_key(self.interface_field.currentText()))
    self.buttonBox.clear()
    self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Ok|QtGui.QDialogButtonBox.Cancel)
    self.interface_field.setFocus(QtCore.Qt.TabFocusReason)
    self.resize(350,80)

  def _fill_interfaces(self):
    if self._interfaces_files is None:
      self.root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
      self._interfaces_files = {}
      for p in self.root_paths:
        ret = self._getInterfaces(p)
        self._interfaces_files = dict(ret.items() + self._interfaces_files.items())
      self.interface_field.clear()
      self.interface_field.clearEditText()
      self.interface_field.addItems(self._interfaces_files.keys())

  def _on_interface_selected(self, interface):
    if self._interfaces_files.has_key(interface):
      self._sync_args = []
      self._sync_args.append(''.join(['_interface_url:=', interface]))
      self.toolButton_EditInterface.setEnabled(True)

  def accept(self):
    if self.textedit.isVisible():
      try:
        tmp_file = os.path.join(nm.screen().LOG_PATH, 'tmp_sync_interface.sync')
        with open(tmp_file, 'w+') as f: 
          iface = f.write(self.textedit.toPlainText())
        from master_sync_fkie.common import read_interface
        read_interface(tmp_file)
        if not self._new_iface and self._interfaces_files.has_key(self.interface_field.currentText()):
          fileName = self._interfaces_files[self.interface_field.currentText()]
        else:
          fileName, selectedFilter = QtGui.QFileDialog.getSaveFileName(self, 'Save sync interface', '/home', "Sync Files (*.sync)")
        if fileName:
          with open(fileName, 'w+') as f: 
            iface = f.write(self.textedit.toPlainText())
            if self._new_iface:
              self.interface_field.clear()
              self._interfaces_files = None
            self._on_select_interface_clicked()
#        QtGui.QDialog.accept(self)
#        self.resetView()
      except Exception as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Create sync interface", 
                          "Error while create interface",
                          str(e)).exec_()
    elif self.interface_field.isVisible():
      interface = self.interface_field.currentText()
      if self._interfaces_files and self._interfaces_files.has_key(interface):
        self._sync_args = []
        self._sync_args.append(''.join(['_interface_url:=', interface]))
        QtGui.QDialog.accept(self)
        self.resetView()
    else:
      QtGui.QDialog.accept(self)
      self.resetView()


  def reject(self):
    if self.textedit.isVisible():
      self._on_select_interface_clicked()
    else:
      QtGui.QDialog.reject(self)
      self.resetView()

  def _on_create_interface_clicked(self):
    self._new_iface = True
    self.interface_field.setVisible(False)
    self.toolButton_CreateInterface.setVisible(False)
    self.toolButton_EditInterface.setVisible(False)
    self.textedit.setVisible(True)
    self.textedit.setText("# The ignore_* lists will be processed first.\n"
                          "ignore_hosts:\n"
                          "sync_hosts:\n\n"
                          "ignore_nodes:\n"
                          "sync_nodes:\n\n"
                          "ignore_topics:\n"
                          "sync_topics:\n\n"
                          "ignore_services:\n"
                          "  - /*get_loggers\n"
                          "  - /*set_logger_level\n"
                          "sync_services:\n\n"
                          "# The sync_topics_on_demand is only regarded, if sync_nodes or sync_topics are set.\n"
                          "# In this case the subscribed and published topics are synchronized even if they are\n"
                          "# not in the sync_* list.\n"
                          "sync_topics_on_demand: False\n"
                          )
    self.resize(350,300)

  def _on_edit_interface_clicked(self):
    self._new_iface = False
    self.interface_field.setVisible(False)
    self.toolButton_CreateInterface.setVisible(False)
    self.toolButton_EditInterface.setVisible(False)
    self.textedit.setVisible(True)
    if self._interfaces_files.has_key(self.interface_field.currentText()):
      try:
        with open(self._interfaces_files[self.interface_field.currentText()], 'rw') as f: 
          iface = f.read()
          self.textedit.setText(iface)
      except Exception as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "Edit sync interface", 
                          "Error while open interface",
                          str(e)).exec_()
    self.resize(350,300)

  def resetView(self):
    self.toolButton_SyncAll.setVisible(True)
    self.toolButton_SyncTopicOnDemand.setVisible(True)
    self.toolButton_SelectInterface.setVisible(True)
    self.interface_field.setVisible(False)
    self.toolButton_CreateInterface.setVisible(False)
    self.toolButton_EditInterface.setVisible(False)
    self.textedit.setVisible(False)
    self.buttonBox.clear()
    self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel)
    self.resize(350,160)

  def _getInterfaces(self, path, package=None):
    result = {}
    if os.path.isdir(path):
      fileList = os.listdir(path)
      # set package, if it is currently None and one found
      if not package:
        # detect package
        if 'manifest.xml' in fileList:
          package = os.path.basename(path)
      for f in fileList:
        ret = self._getInterfaces(os.path.sep.join([path, f]), package)
        result = dict(ret.items() + result.items())
    elif package and os.path.isfile(path) and path.endswith('.sync'):
      # create a selection for binaries
      return {''.join(['pkg://', package, '///', os.path.basename(path)]) : path}
    return result
