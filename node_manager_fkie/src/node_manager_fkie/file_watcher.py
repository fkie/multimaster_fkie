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
  from python_qt_binding import QtCore
except:
  pass

class FileWatcher(QtCore.QObject):
  '''
  A class to watch for file changes.
  '''
  file_changed = QtCore.Signal(str, list)
  '''@ivar: a signal to inform the receiver about the changes on
  launch file or included file. ParameterB{:} (changed file, list of tuples(masteruri, launch file))'''
  
  def __init__(self):
    QtCore.QObject.__init__(self)
    self.file_watcher = QtCore.QFileSystemWatcher()
    self.file_watcher.fileChanged.connect(self.on_file_changed)
    self.changed = {}
    self.launches = {}

  def __del__(self):
    # Delete to avoid segfault if the LaunchConfig class is destroyed recently 
    # after creation and xmlrpclib.ServerProxy process a method call.
    del self.file_watcher

  def on_file_changed(self, file):
    '''
    callback method, which is called by L{QtCore.QFileSystemWatcher} if the 
    launch file or included files are changed. In this case 
    L{FileWatcher.file_changed} signal will be emitted.
    '''
    # to avoid to handle from QFileSystemWatcher fired the signal two times
    if (not self.changed.has_key(file) or (self.changed.has_key(file) and self.changed[file] + 0.05 < time.time())):
      self.changed[file] = time.time()
      changes = []
      for (uri, lfile, _), files in self.launches.items():#_:=id
        if file in files:
          changes.append((uri, lfile))
      self.file_changed.emit(file, changes)

  def add(self, masteruri, launch_file, launch_id, files):
    if self.launches.has_key((masteruri, launch_file, launch_id)):
      self.launches[(masteruri, launch_file, launch_id)].extend([os.path.normpath(f) for f in files])
    else:
      self.launches[(masteruri, launch_file, launch_id)] = [os.path.normpath(f) for f in files]
    self.update_files()

  def rem(self, masteruri, launch_file='', launch_id=''):
    try:
      if launch_file:
        if launch_id:
          del self.launches[(masteruri, launch_file, launch_id)]
        else:
          for (uri, file, id), files in self.launches.items():
            if uri == masteruri and file == launch_file:
              del self.launches[(uri, file, id)]
      else:
        for (uri, file, id), files in self.launches.items():
          if uri == masteruri:
            del self.launches[(uri, file, id)]
    except:
#      import traceback
#      print traceback.format_exc(1)
      pass
    self.update_files()

  def update_files(self):
    result = set()
    for files in self.launches.itervalues():
      result.update(set(files))
    files = self.file_watcher.files()
    if files:
      self.file_watcher.removePaths(files)
    if list(result):
      self.file_watcher.addPaths(list(result))

