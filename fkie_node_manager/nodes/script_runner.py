#!/usr/bin/env python

import os
import shlex
import signal
import subprocess
import sys
import threading
import time

import rospy

from fkie_master_discovery.common import resolve_url

ROS_NODE = 'script_runner'


def set_terminal_name(name):
    '''
    Change the terminal name.

    :param str name: New name of the terminal
    '''
    sys.stdout.write("\x1b]2;%s\x07" % name)


def set_process_name(name):
    '''
    Change the process name.

    :param str name: New process name
    '''
    try:
        from ctypes import cdll, byref, create_string_buffer
        libc = cdll.LoadLibrary('libc.so.6')
        buff = create_string_buffer(len(name) + 1)
        buff.value = name
        libc.prctl(15, byref(buff), 0, 0, 0)
    except Exception:
        pass


class SupervisedPopen(object):
    '''
    The class overrides the subprocess.Popen and waits in a thread for its finish.
    '''

    def __init__(self, args, bufsize=0, executable=None, stdin=None, stdout=None,
                 stderr=subprocess.PIPE, preexec_fn=None, close_fds=False,
                 shell=False, cwd=None, env=None, universal_newlines=False,
                 startupinfo=None, creationflags=0, object_id='', description=''):
        '''
        For arguments see https://docs.python.org/2/library/subprocess.html
        Additional arguments:

        :param str object_id: the identification string of this object and title of the
        error message dialog
        :param str description: the description string used as addiotional information
        in dialog if an error was occured
        '''
        try:
            self._args = args
            self._object_id = object_id
            self._description = description
            # wait for process to avoid 'defunct' processes
            self.popen = subprocess.Popen(args=args, bufsize=bufsize, executable=executable, stdin=stdin, stdout=stdout,
                                          stderr=stderr, preexec_fn=preexec_fn, close_fds=close_fds, shell=shell, cwd=cwd, env=env,
                                          universal_newlines=universal_newlines, startupinfo=startupinfo, creationflags=creationflags)
            thread = threading.Thread(target=self._supervise)
            thread.setDaemon(True)
            thread.start()
        except Exception as _:
            raise

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


class RunThread(threading.Thread):

    def __init__(self, script):
        '''
        :param str masteruri: the URI of the remote ROS master
        '''
        threading.Thread.__init__(self)
        self._script = script
        cmd_list = shlex.split(script)
        self._cmd = []
        for cmd in cmd_list:
            if cmd.startswith("pkg://"):
                resolved = resolve_url(cmd)
                self._cmd.append(resolved)
            else:
                self._cmd.append(cmd)
        self.setDaemon(True)
        self.spopen = None
        self.stopped = False

    def run(self):
        '''
        '''
        try:
            self.spopen = SupervisedPopen(self._cmd)
            while not self.stopped and self.spopen.popen.returncode is None:
                if self.spopen.popen.stderr is not None:
                    reserr = self.spopen.popen.stderr.read()
                    if reserr:
                        rospy.logwarn("script returns follow exception: %s" % reserr.strip())
                    time.sleep(0.1)
            if self.spopen.popen.returncode is not None and self.spopen.popen.returncode != 0:
                rospy.logerr("Script ends with error, code: %d" % self.spopen.popen.returncode)
                os.kill(os.getpid(), signal.SIGKILL)
        except OSError as err:
            rospy.logerr("Error while run '%s': %s" % (self._script, err))
            os.kill(os.getpid(), signal.SIGKILL)
        rospy.loginfo('script finished with code: %d' % self.spopen.popen.returncode)
        rospy.signal_shutdown('script finished with code: %d' % self.spopen.popen.returncode)

    def stop(self, send_sigint=True):
        self.stopped = True
        if send_sigint and self.spopen is not None:
            if self.spopen.popen.pid is not None and self.spopen.popen.returncode is None:
                rospy.loginfo("stop process %d" % self.spopen.popen.pid)
                self.spopen.popen.send_signal(signal.SIGINT)


if __name__ == '__main__':
    name = ROS_NODE
    rospy.init_node(name, log_level=rospy.INFO)
    set_terminal_name(name)
    set_process_name(name)
    param_script = ''
    try:
        param_script = rospy.get_param('~script')
    except KeyError:
        rospy.logerr("No script specified! Use ~script parameter to specify the script!")
        os.kill(os.getpid(), signal.SIGKILL)
    param_stop_script = rospy.get_param('~stop_script', '')
    rospy.loginfo("~script: %s" % param_script)
    rospy.loginfo("~stop_script: %s" % param_stop_script)
    runthread = RunThread(param_script)
    runthread.start()
    rospy.spin()
    # stop the script
    if param_stop_script:
        runthread.stop(False)
        rospy.loginfo("stop using %s" % param_stop_script)
        stopthread = RunThread(param_stop_script)
        stopthread.start()
        stopthread.join(3)
        if stopthread.spopen is not None:
            if stopthread.spopen.popen.stderr is not None:
                reserr = stopthread.spopen.popen.stderr.read()
                if reserr:
                    rospy.logwarn("stop script has follow exception: %s" % reserr)
    else:
        runthread.stop()
        runthread.join(3)
    if runthread.is_alive():
        rospy.logwarn("Script does not stop, try to kill %d..." % runthread.spopen.popen.pid)
        if runthread.spopen is not None:
            runthread.spopen.popen.send_signal(signal.SIGKILL)
        if runthread.spopen is not None:
            runthread.spopen.popen.send_signal(signal.SIGTERM)
