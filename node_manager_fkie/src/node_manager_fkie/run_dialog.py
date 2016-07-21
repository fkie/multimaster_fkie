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

from python_qt_binding.QtCore import Qt, QMetaObject
try:
    from python_qt_binding.QtGui import QComboBox, QDialog, QDialogButtonBox, QLabel, QLineEdit, QWidget
    from python_qt_binding.QtGui import QFormLayout, QHBoxLayout, QVBoxLayout, QSizePolicy
except:
    from python_qt_binding.QtWidgets import QComboBox, QDialog, QDialogButtonBox, QLabel, QLineEdit, QWidget
    from python_qt_binding.QtWidgets import QFormLayout, QHBoxLayout, QVBoxLayout, QSizePolicy
import os

from packages_thread import PackagesThread
import node_manager_fkie as nm


class PackageDialog(QDialog):

    def __init__(self, parent=None):
        QDialog.__init__(self, parent)
        self.setWindowTitle('Select Binary')
        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")

        self.content = QWidget()
        self.contentLayout = QFormLayout(self.content)
        self.contentLayout.setVerticalSpacing(0)
        self.verticalLayout.addWidget(self.content)

        self.packages = None

        package_label = QLabel("Package:", self.content)
        self.package_field = QComboBox(self.content)
        self.package_field.setInsertPolicy(QComboBox.InsertAlphabetically)
        self.package_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.package_field.setEditable(True)
        self.contentLayout.addRow(package_label, self.package_field)
        binary_label = QLabel("Binary:", self.content)
        self.binary_field = QComboBox(self.content)
#    self.binary_field.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.binary_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.binary_field.setEditable(True)
        self.contentLayout.addRow(binary_label, self.binary_field)

        self.buttonBox = QDialogButtonBox(self)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)

        self.package_field.setFocus(Qt.TabFocusReason)
        self.package = ''
        self.binary = ''

        if self.packages is None:
            self.package_field.addItems(['packages searching...'])
            self.package_field.setCurrentIndex(0)
            self._fill_packages_thread = PackagesThread()
            self._fill_packages_thread.packages.connect(self._fill_packages)
            self._fill_packages_thread.start()

        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        QMetaObject.connectSlotsByName(self)
        self.package_field.activated[str].connect(self.on_package_selected)
        if hasattr(self.package_field, "textChanged"):  # qt compatibility
            self.package_field.textChanged.connect(self.on_package_selected)
            self.binary_field.textChanged.connect(self.on_binary_selected)
        else:
            self.package_field.editTextChanged.connect(self.on_package_selected)
            self.binary_field.editTextChanged.connect(self.on_binary_selected)

    def _fill_packages(self, packages):
        # fill the input fields
        self.packages = packages
        packages = packages.keys()
        packages.sort()
        self.package_field.clear()
        self.package_field.clearEditText()
        self.package_field.addItems(packages)

    def _getBinaries(self, path):
        result = {}
        if os.path.isdir(path):
            fileList = os.listdir(path)
            for f in fileList:
                if f and f[0] != '.' and f not in ['build'] and not f.endswith('.cfg') and not f.endswith('.so'):
                    ret = self._getBinaries(os.path.join(path, f))
                    result = dict(ret.items() + result.items())
        elif os.path.isfile(path) and os.access(path, os.X_OK):
            # create a selection for binaries
            return {os.path.basename(path): path}
        return result

    def on_package_selected(self, package):
        self.binary_field.clear()
        if self.packages and package in self.packages:
            self.binary_field.setEnabled(True)
            path = self.packages[package]
            binaries = self._getBinaries(path).keys()
            try:
                # find binaries in catkin workspace
                from catkin.find_in_workspaces import find_in_workspaces as catkin_find
                search_paths = catkin_find(search_dirs=['libexec', 'share'], project=package, first_matching_workspace_only=True)
                for p in search_paths:
                    binaries += self._getBinaries(p).keys()
            except:
                pass
            binaries = list(set(binaries))
            binaries.sort()
            self.binary_field.addItems(binaries)
            self.package = package
            self.binary = self.binary_field.currentText()

    def on_binary_selected(self, binary):
        self.binary = binary


class RunDialog(PackageDialog):
    '''
    A dialog to run a ROS node without configuration
    '''

    def __init__(self, host, masteruri=None, parent=None):
        PackageDialog.__init__(self, parent)
        self.host = host
        self.setWindowTitle('Run')

        ns_name_label = QLabel("NS/Name:", self.content)
        self.ns_field = QComboBox(self.content)
        self.ns_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.ns_field.setEditable(True)
        ns_history = nm.history().cachedParamValues('run_dialog/NS')
        ns_history.insert(0, '/')
        self.ns_field.addItems(ns_history)
        self.name_field = QLineEdit(self.content)
        self.name_field.setEnabled(False)
        horizontalLayout = QHBoxLayout()
        horizontalLayout.addWidget(self.ns_field)
        horizontalLayout.addWidget(self.name_field)
        self.contentLayout.addRow(ns_name_label, horizontalLayout)
        args_label = QLabel("Args:", self.content)
        self.args_field = QComboBox(self.content)
        self.args_field.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLength)
        self.args_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.args_field.setEditable(True)
        self.contentLayout.addRow(args_label, self.args_field)
        args_history = nm.history().cachedParamValues('run_dialog/Args')
        args_history.insert(0, '')
        self.args_field.addItems(args_history)

        host_label = QLabel("Host:", self.content)
        self.host_field = QComboBox(self.content)
#    self.host_field.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.host_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.host_field.setEditable(True)
        host_label.setBuddy(self.host_field)
        self.contentLayout.addRow(host_label, self.host_field)
        self.host_history = host_history = nm.history().cachedParamValues('/Host')
        if self.host in host_history:
            host_history.remove(self.host)
        host_history.insert(0, self.host)
        self.host_field.addItems(host_history)

        master_label = QLabel("ROS Master URI:", self.content)
        self.master_field = QComboBox(self.content)
        self.master_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.master_field.setEditable(True)
        master_label.setBuddy(self.host_field)
        self.contentLayout.addRow(master_label, self.master_field)
        self.master_history = master_history = nm.history().cachedParamValues('/Optional Parameter/ROS Master URI')
        self.masteruri = "ROS_MASTER_URI" if masteruri is None else masteruri
        if self.masteruri in master_history:
            master_history.remove(self.masteruri)
        master_history.insert(0, self.masteruri)
        self.master_field.addItems(master_history)

#    self.package_field.setFocus(QtCore.Qt.TabFocusReason)
        if hasattr(self.package_field, "textChanged"):  # qt compatibility
            self.package_field.textChanged.connect(self.on_package_selected)
        else:
            self.package_field.editTextChanged.connect(self.on_package_selected)
        self.binary_field.activated[str].connect(self.on_binary_selected)

    def run_params(self):
        '''
        Runs the selected node, or do nothing.
        :return: a tuple with host, package, binary, name, args, maseruri or empty tuple on errors
        '''
        self.binary = self.binary_field.currentText()
        self.host = self.host_field.currentText() if self.host_field.currentText() else self.host
        self.masteruri = self.master_field.currentText() if self.master_field.currentText() else self.masteruri
        if self.host not in self.host_history and self.host != 'localhost' and self.host != '127.0.0.1':
            nm.history().add2HostHistory(self.host)
        ns = self.ns_field.currentText()
        if ns and ns != '/':
            nm.history().addParamCache('run_dialog/NS', ns)
        args = self.args_field.currentText()
        if args:
            nm.history().addParamCache('run_dialog/Args', args)
        if self.package and self.binary:
            nm.history().addParamCache('/Host', self.host)
            return (self.host, self.package, self.binary, self.name_field.text(), ('__ns:=%s %s' % (ns, args)).split(' '), None if self.masteruri == 'ROS_MASTER_URI' else self.masteruri)
        return ()

    def on_package_selected(self, package):
        PackageDialog.on_package_selected(self, package)
        if self.packages and package in self.packages:
            self.args_field.setEnabled(True)
            self.ns_field.setEnabled(True)
            self.name_field.setEnabled(True)
            root, _ext = os.path.splitext(os.path.basename(self.binary_field.currentText()))
            self.name_field.setText(root)

    def on_binary_selected(self, binary):
        root, _ext = os.path.splitext(os.path.basename(binary))
        self.name_field.setText(root)
