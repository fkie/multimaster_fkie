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

import os
import time

try:
    from python_qt_binding.QtCore import QObject, Signal, QFileSystemWatcher
except:
    pass


class FileWatcher(QObject):
    '''
    A class to watch for file changes.
    '''
    config_changed = Signal(str, list)
    '''@ivar: a signal to inform the receiver about the changes on
    launch file or included file.
    ParameterB{:} (changed file, list of tuples(masteruri, launch file))'''

    binary_changed = Signal(str, list)
    '''@ivar: a signal to inform the receiver about the changes on
    binary file or included file.
    ParameterB{:} (binary file, list of tuples(node name, masteruri, launchfile))
    '''

    def __init__(self):
        QObject.__init__(self)
        self.file_watcher = QFileSystemWatcher()
        self.file_watcher.fileChanged.connect(self.on_file_changed)
        self.changed = {}
        self.launches = {}
        self.binaries = {}

    def __del__(self):
        # Delete to avoid segfault if the LaunchConfig class is destroyed recently
        # after creation and xmlrpclib.ServerProxy process a method call.
        del self.file_watcher

    def on_file_changed(self, filepath):
        '''
        callback method, which is called by U{QtCore.QFileSystemWatcher<https://srinikom.github.io/pyside-docs/PySide/QtCore/QFileSystemWatcher.html>}
        if one of a launch file, included files or binary are changed.
        Depend on type of the file a L{FileWatcher.config_changed} or
        L{FileWatcher.binary_changed} signal will be emitted.
        '''
        # to avoid to handle from QFileSystemWatcher fired the signal two times
        if (filepath not in self.changed or (filepath in self.changed and self.changed[filepath] + 0.05 < time.time())):
            self.changed[filepath] = time.time()
            launch_changes = []
            for (uri, lfile, _), files in self.launches.items():  # _:=id
                if filepath in files:
                    launch_changes.append((uri, lfile))
            if launch_changes:
                self.config_changed.emit(filepath, launch_changes)
            binaries_changed = []
            for node_name, (binary_file, masteruri, launchfile) in self.binaries.items():
                if filepath == binary_file:
                    binaries_changed.append((node_name, masteruri, launchfile))
            if binaries_changed:
                self.binary_changed.emit(filepath, binaries_changed)

    def add_launch(self, masteruri, launch_file, launch_id, files):
        if (masteruri, launch_file, launch_id) in self.launches:
            self.launches[(masteruri, launch_file, launch_id)].extend([os.path.normpath(f) for f in files])
        else:
            self.launches[(masteruri, launch_file, launch_id)] = [os.path.normpath(f) for f in files]
        self.update_files()

    def rem_launch(self, masteruri, launch_file='', launch_id=''):
        try:
            if launch_file:
                if launch_id:
                    del self.launches[(masteruri, launch_file, launch_id)]
                else:
                    for (uri, cfgfile, cfgid), _ in self.launches.items():
                        if uri == masteruri and cfgfile == launch_file:
                            del self.launches[(uri, cfgfile, cfgid)]
            else:
                for (uri, cfgfile, cfgid), _ in self.launches.items():
                    if uri == masteruri:
                        del self.launches[(uri, cfgfile, cfgid)]
        except:
            pass
        self.update_files()

    def add_binary(self, binary_file, node_name, masteruri, launchfile):
        self.binaries[node_name] = (binary_file, masteruri, launchfile)
        self.update_files()

    def rem_binary(self, node_name):
        try:
            del self.binaries[node_name]
            self.update_files()
        except:
            pass

    def update_files(self):
        result = set()
        for files in self.launches.itervalues():
            result.update(set(files))
        binaries = set()
        for _, (binary_file, _, _) in self.binaries.items():
            binaries.add(binary_file)
        result.update(binaries)
        files = set(self.file_watcher.files())
        to_remove = list(files - result)
        if to_remove:
            self.file_watcher.removePaths(to_remove)
        to_add = list(result - files)
        if to_add:
            self.file_watcher.addPaths(to_add)
