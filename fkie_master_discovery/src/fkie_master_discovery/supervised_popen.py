# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
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



import rospy
import subprocess
import threading


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
        rospy.logdebug("start job [%s]" % self._object_id)
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
            rospy.logwarn('%s - %s: %s' % (self._object_id, self._description, result_err))
        rospy.logdebug("job [%s] finished" % self._object_id)
