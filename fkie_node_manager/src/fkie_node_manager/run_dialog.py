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



import threading
from python_qt_binding.QtCore import QObject, Qt, QMetaObject, Signal
try:
    from python_qt_binding.QtGui import QComboBox, QDialog, QDialogButtonBox, QLabel, QLineEdit, QWidget
    from python_qt_binding.QtGui import QFormLayout, QHBoxLayout, QVBoxLayout, QSizePolicy
except Exception:
    from python_qt_binding.QtWidgets import QComboBox, QDialog, QDialogButtonBox, QLabel, QLineEdit, QWidget
    from python_qt_binding.QtWidgets import QFormLayout, QHBoxLayout, QVBoxLayout, QSizePolicy
import os

from fkie_node_manager_daemon import url as nmdurl
import fkie_node_manager as nm


class RequestBinariesThread(QObject, threading.Thread):
    '''
    A thread to to retrieve the binaries for a given package
    and publish it by sending a QT signal.
    '''
    binaries_signal = Signal(str, dict)

    def __init__(self, package, grpc_url, parent=None):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._grpc_url = grpc_url
        self._package = package
        self.setDaemon(True)
        self._canceled = False
        self._result = {}

    def cancel(self):
        self._canceled = True

    @property
    def pkgname(self):
        return self._package

    def reemit(self):
        self.binaries_signal.emit(self._package, self._result)

    def run(self):
        '''
        '''
        if self._grpc_url:
            try:
                self._result = nm.nmd().file.get_package_binaries(self._package, nmdurl.nmduri(self._grpc_url))
                if not self._canceled:
                    self.binaries_signal.emit(self._package, self._result)
            except Exception:
                import traceback
                print(traceback.format_exc())


class PackageDialog(QDialog):

    def __init__(self, masteruri, parent=None):
        QDialog.__init__(self, parent)
        self.setWindowTitle('Select Binary')
        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")

        self.content = QWidget()
        self.contentLayout = QFormLayout(self.content)
        self.contentLayout.setVerticalSpacing(0)
        self.verticalLayout.addWidget(self.content)

        self.packages = None
        self.masteruri = "ROS_MASTER_URI" if masteruri is None else masteruri
        package_label = QLabel("Package:", self.content)
        self.package_field = QComboBox(self.content)
        self.package_field.setInsertPolicy(QComboBox.InsertAlphabetically)
        self.package_field.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
        self.package_field.setEditable(True)
        self.contentLayout.addRow(package_label, self.package_field)
        binary_label = QLabel("Binary:", self.content)
        self.binary_field = QComboBox(self.content)
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
        self._request_bin_thread = None

        if self.packages is None:
            self.package_field.addItems(['packages searching...'])
            self.package_field.setCurrentIndex(0)
        # fill the input fields
        self.packages = {name: path for path, name in nm.nmd().file.get_packages(nmdurl.nmduri(masteruri)).items()}
        packages = list(self.packages.keys())
        packages.sort()
        self.package_field.clear()
        self.package_field.clearEditText()
        self.package_field.addItems(packages)
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

    def on_package_selected(self, package):
        getnew = False
        if self._request_bin_thread is None:
            getnew = True
        else:
            if self._request_bin_thread.pkgname != package:
                self._request_bin_thread.cancel()
                getnew = True
        if self._request_bin_thread is not None and self._request_bin_thread.pkgname == package:
            # use already got data
            self._request_bin_thread.reemit()
        elif getnew:
            self.binary_field.clear()
            if self.packages and package in self.packages:
                self.binary_field.setEnabled(True)
                self._request_bin_thread = RequestBinariesThread(package, nmdurl.nmduri(self.masteruri))
                self._request_bin_thread.binaries_signal.connect(self._on_new_binaries)
                self._request_bin_thread.start()

    def _on_new_binaries(self, pkgname, binaries):
        # update the binaries
        binaries = [os.path.basename(item) for item in binaries.keys()]
        binaries = list(set(binaries))
        binaries.sort()
        self.binary_field.addItems(binaries)
        self.package = pkgname
        self.binary = self.binary_field.currentText()

    def on_binary_selected(self, binary):
        self.binary = binary


class RunDialog(PackageDialog):
    '''
    A dialog to run a ROS node without configuration
    '''

    def __init__(self, host, masteruri=None, parent=None):
        PackageDialog.__init__(self, masteruri, parent)
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
        if self.masteruri in master_history:
            master_history.remove(self.masteruri)
        master_history.insert(0, self.masteruri)
        self.master_field.addItems(master_history)
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
            result = {'host': self.host,
                      'package': self.package,
                      'binary': self.binary,
                      'name': self.name_field.text(),
                      'args': ('__ns:=%s %s' % (ns, args)).split(' '),
                      'masteruri': None if self.masteruri == 'ROS_MASTER_URI' else self.masteruri}
            return result
        return {}

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
