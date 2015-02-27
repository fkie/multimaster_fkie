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
import sys
import subprocess
import threading
import roslib

from python_qt_binding import QtCore, QtGui

import node_manager_fkie as nm
from detailed_msg_box import WarningMessageBox, DetailedError


class SupervisedPopen(QtCore.QObject, subprocess.Popen):
  '''
  The class overrides the subprocess.Popen and waits in a thread for its finish.
  If an error is printed out, it will be shown in a message dialog.
  '''
  error = QtCore.Signal(str, str, str)
  '''@ivar: the signal is emitted if error output was detected (id, decription, message)'''

  finished = QtCore.Signal(str)
  '''@ivar: the signal is emitted on exit (id)'''

  def __init__(self, args, bufsize=0, executable=None, stdin=None, stdout=None, stderr=subprocess.PIPE, preexec_fn=None, close_fds=False, shell=False, cwd=None, env=None, universal_newlines=False, startupinfo=None, creationflags=0, id='', description=''):
    '''
    For arguments see https://docs.python.org/2/library/subprocess.html
    Additional arguments:
    :param id: the identification string of this object and title of the error message dialog 
    :type id: str
    :param description: the description string used as addiotional information 
                        in dialog if an error was occured
    :type description: str
    '''
    try:
      subprocess.Popen.__init__(self, args, bufsize, executable, stdin, stdout, stderr, preexec_fn, close_fds, shell, cwd, env, universal_newlines, startupinfo, creationflags)
      QtCore.QObject.__init__(self)
      self._args = args
      self._id = id
      self._description = description
      self.error.connect(self.on_error)
      # wait for process to avoid 'defunct' processes
      thread = threading.Thread(target=self.supervise)
      thread.setDaemon(True)
      thread.start()
    except Exception as e:
      raise

#   def __del__(self):
#     print "Deleted:", self._description

  def supervise(self):
    # wait for process to avoid 'defunct' processes
    self.wait()
    result_err = ''
    if not self.stderr is None:
      result_err = self.stderr.read()
    if result_err:
      self.error.emit(self._id, self._description, result_err)
    self.finished.emit(self._id)

  def on_error(self, id, descr, msg):
    WarningMessageBox(QtGui.QMessageBox.Warning, id, '%s\n\n%s'%(descr, msg), ' '.join(self._args)).exec_()

