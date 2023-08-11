# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess
import threading
from fkie_multimaster_pylib.logging.logging import Log


class SupervisedPopen():
    '''
    The class overrides the subprocess.Popen and waits in a thread for its finish.
    If an error is printed out.
    '''

    def __init__(self, args, bufsize=0, executable=None, stdin=None, stdout=None,
                 stderr=subprocess.PIPE, preexec_fn=None, close_fds=False,
                 shell=False, cwd=None, env=None, universal_newlines=False,
                 startupinfo=None, creationflags=0, object_id='', description=''):
        '''
        For arguments see https://docs.python.org/2/library/subprocess.html
        Additional arguments:

        :param str object_id: the identification string of this object and title of the error message dialog
        :param str description: the description string used as additional information in dialog if an error was occurred
        '''
        self._args = args
        self._object_id = object_id
        Log.debug("start job [%s]" % self._object_id)
        self._description = description
        # wait for process to avoid 'defunct' processes
        self.popen = subprocess.Popen(args=args, bufsize=bufsize, executable=executable, stdin=stdin, stdout=stdout,
                                      stderr=stderr, preexec_fn=preexec_fn, close_fds=close_fds, shell=shell, cwd=cwd, env=env,
                                      universal_newlines=universal_newlines, startupinfo=startupinfo, creationflags=creationflags)
        thread = threading.Thread(target=self._supervise)
        thread.setDaemon(True)
        thread.start()

#   def __del__(self):
#     print "Deleted:", self._description

    @property
    def stdout(self):
        return self.popen.stdout

    @property
    def stderr(self):
        return self.popen.stderr

    @property
    def stdin(self):
        return self.popen.stdin

    def _supervise(self):
        '''
        Wait for process to avoid 'defunct' processes
        '''
        self.popen.wait()
        result_err = ''
        if self.stderr is not None:
            result_err = self.stderr.read()
        if result_err:
            Log.warn('%s - %s: %s' %
                     (self._object_id, self._description, result_err))
        Log.debug("job [%s] finished" % self._object_id)
