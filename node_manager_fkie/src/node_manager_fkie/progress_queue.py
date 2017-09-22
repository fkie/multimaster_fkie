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

from python_qt_binding.QtCore import QObject, Signal
import threading

import rospy

from node_manager_fkie.common import utf8
from node_manager_fkie.detailed_msg_box import MessageBox, DetailedError
import node_manager_fkie as nm


class InteractionNeededError(Exception):
    '''
    request: AuthenticationRequest
    '''

    def __init__(self, request, method, args):
        Exception.__init__(self)
        self.method = method
        self.request = request
        self.args = args

    def __str__(self):
        return "InteractionNeededError"


class ProgressQueue(QObject):
    '''
    The queue provides a threaded execution of given tasks.
    '''

    def __init__(self, progress_frame, progress_bar, progress_cancel_button, name=''):
        QObject.__init__(self)
        self.__progress_queue = []
        self.__running = False
        self._name = name
        self._progress_frame = progress_frame
        self._progress_bar = progress_bar
        self._progress_cancel_button = progress_cancel_button
        progress_frame.setVisible(False)
        progress_cancel_button.clicked.connect(self._on_progress_canceled)

    def stop(self):
        '''
        Deletes all queued tasks and wait 3 seconds for the end of current running
        thread.
        '''
        try:
            val = self._progress_bar.value()
            if val < len(self.__progress_queue):
                print "  Stop progress queue '%s'..." % self._name
                thread = self.__progress_queue[val]
                self.__progress_queue = []
                if thread.is_alive():
                    thread.join(3)
                print "  Progress queue '%s' stopped!" % self._name
        except Exception:
            import traceback
            print utf8(traceback.format_exc())

    def add2queue(self, ident, descr, target=None, args=()):
        '''
        Adds new task to the queue. After the task was added you need call start().
        :param ident: the unique identification string
        :type ident: str
        :param descr: the description of the task
        :type descr: str
        :param target: is the callable object to be invoked in a new thread.
        Defaults to None, meaning nothing is called.
        :param args: is the argument tuple for the target invocation. Defaults to ()
        '''
        pthread = ProgressThread(ident, descr, target, args)
        pthread.finished_signal.connect(self._progress_thread_finished)
        pthread.error_signal.connect(self._progress_thread_error)
        pthread.request_interact_signal.connect(self._on_request_interact)
        self.__progress_queue.append(pthread)
        self._progress_bar.setMaximum(len(self.__progress_queue))

    def start(self):
        '''
        Starts the execution of tasks in the queue.
        '''
        if not self.__running and self.__progress_queue:
            self._progress_frame.setVisible(True)
            self.__running = True
            self._progress_bar.setToolTip(self.__progress_queue[0].descr)
            dscr_len = self._progress_bar.size().width() / 10
            self._progress_bar.setFormat(''.join(['%v/%m - ', self.__progress_queue[0].descr[0:dscr_len]]))
            self._progress_bar.setValue(0)
            self.__progress_queue[0].start()

    def count(self):
        '''
        :return: the count of tasks in the queue
        :rtype: int
        '''
        return len(self.__progress_queue)

    def has_id(self, ident):
        '''
        Searches the current and planed threads for given id and returns `True` if
        one is found.
        '''
        for thread in self.__progress_queue:
            if thread.id() == ident:
                return True
        return False

    def _progress_thread_finished(self, ident):
        try:
            val = self._progress_bar.value()
            # be on the safe side that the finished thread is the first thread in the
            # queue (avoid calls from canceled threads)
            if ident == self.__progress_queue[val].id():
                val = val + 1
            th = self.__progress_queue[val]
            self._progress_bar.setToolTip(th.descr)
            dscr_len = self._progress_bar.size().width() / 10
            self._progress_bar.setFormat(''.join(['%v/%m - ', th.descr[0:dscr_len]]))
            self.__progress_queue[val].start()
            self._progress_bar.setValue(val)
            # print "PG finished ok", id
        except:
            # print "PG finished err", id
            for thread in self.__progress_queue:
                thread.join(1)
            self._progress_frame.setVisible(False)
            self.__running = False
            # print "PG finished delete all..."
            self.__progress_queue = []
            # print "PG finished delete all ok"

    def _progress_thread_error(self, ident, title, msg, detailed_msg):
        btns = (MessageBox.Ignore | MessageBox.Avoid)
        if len(self.__progress_queue) > 1:
            btns = (btns | MessageBox.Abort)
        res = MessageBox(MessageBox.Critical, title, msg, detailed_msg, buttons=btns).exec_()
        if res == MessageBox.Abort:
            self.__progress_queue = []
            self._progress_frame.setVisible(False)
            self.__running = False
        else:
            self._progress_thread_finished(ident)

    def _on_progress_canceled(self):
        try:
            if self.__progress_queue:
                try:
                    pthread = self.__progress_queue[self._progress_bar.value()]
                    pthread.finished_signal.disconnect(self._progress_thread_finished)
                    pthread.error_signal.disconnect(self._progress_thread_error)
                    pthread.request_interact_signal.disconnect(self._on_request_interact)
#          self.__progress_queue[self._progress_bar.value()].terminate()
                except:
                    pass
            self.__progress_queue = []
            self._progress_frame.setVisible(False)
            self.__running = False
        except:
            import traceback
            print utf8(traceback.format_exc(1))

    def _on_request_interact(self, ident, descr, req):
        '''
        If the interaction of the user is needed a message dialog must be exceuted
        in the main Qt thread. The requests are done by different request exceptinos.
        These are handled by this method.
        '''
        if isinstance(req.request, nm.AuthenticationRequest):
            res, user, pw = nm.ssh()._requestPW(req.request.user, req.request.host)
            if not res:
                self._on_progress_canceled()
                return
            if req.args and isinstance(req.args[0], nm.AdvRunCfg):
                req.args[0].user = user
                req.args[0].pw = pw
            else:
                req.args = req.args + (user, pw)
            pt = ProgressThread(ident, descr, req.method, (req.args))
            pt.finished_signal.connect(self._progress_thread_finished)
            pt.error_signal.connect(self._progress_thread_error)
            pt.request_interact_signal.connect(self._on_request_interact)
            pt.start()
        elif isinstance(req.request, nm.ScreenSelectionRequest):
            from node_manager_fkie.select_dialog import SelectDialog
            items, _ = SelectDialog.getValue('Show screen', '', req.request.choices.keys(), False)
            if not items:
                self._progress_thread_finished(ident)
                return
            res = [req.request.choices[i] for i in items]
            pt = ProgressThread(ident, descr, req.method, (req.args + (res,)))
            pt.finished_signal.connect(self._progress_thread_finished)
            pt.error_signal.connect(self._progress_thread_error)
            pt.request_interact_signal.connect(self._on_request_interact)
            pt.start()
        elif isinstance(req.request, nm.BinarySelectionRequest):
            from node_manager_fkie.select_dialog import SelectDialog
            items, _ = SelectDialog.getValue('Multiple executables', '', req.request.choices, True)
            if not items:
                self._progress_thread_finished(ident)
                return
            res = items[0]
            if req.args and isinstance(req.args[0], nm.AdvRunCfg):
                req.args[0].executable = res
            else:
                req.args = req.args + (res,)
            pt = ProgressThread(ident, descr, req.method, (req.args))
            pt.finished_signal.connect(self._progress_thread_finished)
            pt.error_signal.connect(self._progress_thread_error)
            pt.request_interact_signal.connect(self._on_request_interact)
            pt.start()
        elif isinstance(req.request, nm.LaunchArgsSelectionRequest):
            from node_manager_fkie.parameter_dialog import ParameterDialog
            input_dia = ParameterDialog(req.request.args_dict)
            input_dia.setFilterVisible(False)
            input_dia.setWindowTitle('Enter the argv for %s' % req.request.launchfile)
            if input_dia.exec_():
                params = input_dia.getKeywords()
                argv = []
                for prm, val in params.items():
                    if val:
                        argv.append('%s:=%s' % (prm, val))
                res = argv
                pt = ProgressThread(ident, descr, req.method, (req.args + (argv,)))
                pt.finished_signal.connect(self._progress_thread_finished)
                pt.error_signal.connect(self._progress_thread_error)
                pt.request_interact_signal.connect(self._on_request_interact)
                pt.start()
            else:
                self._progress_thread_finished(ident)
                return


class ProgressThread(QObject, threading.Thread):
    '''
    A thread to execute a method in a thread.
    '''
    finished_signal = Signal(str)
    '''
  @ivar: finished_signal is a signal, which is emitted, if the thread is finished.
  '''

    error_signal = Signal(str, str, str, str)
    '''
  @ivar: error_signal is a signal (id, title, error message, detailed error message),
  which is emitted, if an error while run of the thread was occurred.
  '''

    request_interact_signal = Signal(str, str, InteractionNeededError)

    def __init__(self, ident, descr='', target=None, args=()):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._id = ident
        self.descr = descr
        self._target = target
        self._args = args
        self.setDaemon(True)

    def id(self):
        return self._id

    def run(self):
        '''
        '''
        try:
            if self._target is not None:
                if 'pqid' in self._target.func_code.co_varnames:
                    self._target(*self._args, pqid=self._id)
                else:
                    self._target(*self._args)
                self.finished_signal.emit(self._id)
            else:
                self.error_signal.emit(self._id, 'No target specified')
        except InteractionNeededError as ine:
            self.request_interact_signal.emit(self._id, self.descr, ine)
        except DetailedError as err:
            self.error_signal.emit(self._id, err.title, err.value, err.detailed_text)
        except:
            import traceback
#      print traceback.print_exc()
            formatted_lines = traceback.format_exc(1).splitlines()
            last_line = formatted_lines[-1]
            index = 1
            while not last_line and len(formatted_lines) > index:
                index += 1
                last_line = formatted_lines[-index]
            self.error_signal.emit(self._id, 'Progress Job Error',
                                   "%s failed:\n%s" % (utf8(self.descr), utf8(last_line)),
                                   utf8(traceback.format_exc(4)))
            rospy.logwarn("%s failed:\n\t%s", utf8(self.descr), utf8(last_line))
