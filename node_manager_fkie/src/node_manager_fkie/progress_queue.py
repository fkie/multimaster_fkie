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
from python_qt_binding import QtCore
from python_qt_binding import QtGui

import rospy

import node_manager_fkie as nm
from detailed_msg_box import WarningMessageBox, DetailedError

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


class ProgressQueue(QtCore.QObject):

  def __init__(self, progress_frame, progress_bar, progress_cancel_button):
    QtCore.QObject.__init__(self)
    self.__ignore_err_list = []
    self.__progress_queue = []
    self.__running = False
    self._progress_frame = progress_frame
    self._progress_bar = progress_bar
    self._progress_cancel_button = progress_cancel_button
    progress_frame.setVisible(False)
    progress_cancel_button.clicked.connect(self._on_progress_canceled)

  def stop(self):
    try:
      val = self._progress_bar.value()
      if val < len(self.__progress_queue):
        print "  Shutdown progress queue..."
        thread = self.__progress_queue[val]
        self.__progress_queue = []
        thread.join(3)
        print "  Progress queue is off!"
    except:
      pass


  def add2queue(self, ident, descr, target=None, args=()):
    pt = ProgressThread(ident, descr, target, args)
    pt.finished_signal.connect(self._progress_thread_finished)
    pt.error_signal.connect(self._progress_thread_error)
    pt.request_interact_signal.connect(self._on_request_interact)
    self.__progress_queue.append(pt)
    self._progress_bar.setMaximum(len(self.__progress_queue))

  def start(self):
    if not self.__running and self.__progress_queue:
      self._progress_frame.setVisible(True)
      self.__running = True
      self._progress_bar.setToolTip(self.__progress_queue[0].descr)
      dscr_len = self._progress_bar.size().width()/10
      self._progress_bar.setFormat(''.join(['%v/%m - ', self.__progress_queue[0].descr[0:dscr_len]]))
      self._progress_bar.setValue(0)
      self.__progress_queue[0].start()

  def count(self):
    return len(self.__progress_queue)

  def has_id(self, ident):
    '''
    Searches the current and planed threads for given id and returns `True` if 
    one is found.
    '''
    for th in self.__progress_queue:
      if th.id() == ident:
        return True
    return False

  def _progress_thread_finished(self, ident):
    try:
      val = self._progress_bar.value()
      # be on the safe side that the finished thread is the first thread in the queue (avoid calls from canceled threads)
      if ident == self.__progress_queue[val].id():
        val = val + 1
      th = self.__progress_queue[val]
      self._progress_bar.setToolTip(th.descr)
      dscr_len = self._progress_bar.size().width()/10
      self._progress_bar.setFormat(''.join(['%v/%m - ', th.descr[0:dscr_len]]))
      self.__progress_queue[val].start()
      self._progress_bar.setValue(val)
      #'print "PG finished ok", id
    except:
      #'print "PG finished err", id
      for thread in self.__progress_queue:
        thread.join(1)
      self._progress_frame.setVisible(False)
      self.__running = False
      #'print "PG finished delete all..."
      self.__progress_queue = []
      #'print "PG finished delete all ok"

  def _progress_thread_error(self, ident, title, msg, detailed_msg):
    if detailed_msg in self.__ignore_err_list:
      self._progress_thread_finished(ident)
      return
    btns = (QtGui.QMessageBox.Ignore)
    if len(self.__progress_queue) > 1:
      btns = (QtGui.QMessageBox.Ignore|QtGui.QMessageBox.Abort)
    res = WarningMessageBox(QtGui.QMessageBox.Warning, title, msg, detailed_msg,
                            buttons=btns ).exec_()
    if res == QtGui.QMessageBox.Abort:
      self.__progress_queue = []
      self._progress_frame.setVisible(False)
      self.__running = False
    else:
      if res == 1 or res == 0: # HACK: the number is returned, if "Don't show again" is pressed, instead 'QtGui.QMessageBox.HelpRole' (4)
        self.__ignore_err_list.append(detailed_msg)
      self._progress_thread_finished(ident)

  def _on_progress_canceled(self):
    try:
#      self.__progress_queue[self._progress_bar.value()].wait()
      if self.__progress_queue:
        try:
          self.__progress_queue[self._progress_bar.value()].finished_signal.disconnect(self._progress_thread_finished)
          self.__progress_queue[self._progress_bar.value()].error_signal.disconnect(self._progress_thread_error)
          self.__progress_queue[self._progress_bar.value()].request_interact_signal.disconnect(self._on_request_interact)
#          self.__progress_queue[self._progress_bar.value()].terminate()
        except:
#          print str(self.__progress_queue[self._progress_bar.value()].getName()), 'could not be terminated'
          pass
      self.__progress_queue = []
      self._progress_frame.setVisible(False)
      self.__running = False
    except:
      import traceback
      print traceback.format_exc(1)

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
      pt = ProgressThread(ident, descr, req.method, (req.args+(user, pw)))
      pt.finished_signal.connect(self._progress_thread_finished)
      pt.error_signal.connect(self._progress_thread_error)
      pt.request_interact_signal.connect(self._on_request_interact)
      pt.start()
    elif isinstance(req.request, nm.ScreenSelectionRequest):
      from select_dialog import SelectDialog
      items, _ = SelectDialog.getValue('Show screen', '', req.request.choices.keys(), False)
      if not items:
        self._progress_thread_finished(ident)
        return
      res = [req.request.choices[i] for i in items]
      pt = ProgressThread(ident, descr, req.method, (req.args+(res,)))
      pt.finished_signal.connect(self._progress_thread_finished)
      pt.error_signal.connect(self._progress_thread_error)
      pt.request_interact_signal.connect(self._on_request_interact)
      pt.start()
    elif isinstance(req.request, nm.BinarySelectionRequest):
      from select_dialog import SelectDialog
      items, _ = SelectDialog.getValue('Multiple executables', '', req.request.choices, True)
      if not items:
        self._progress_thread_finished(ident)
        return
      res = items[0]
      pt = ProgressThread(ident, descr, req.method, (req.args+(res,)))
      pt.finished_signal.connect(self._progress_thread_finished)
      pt.error_signal.connect(self._progress_thread_error)
      pt.request_interact_signal.connect(self._on_request_interact)
      pt.start()
    elif isinstance(req.request, nm.LaunchArgsSelectionRequest):
      from parameter_dialog import ParameterDialog
      inputDia = ParameterDialog(req.request.args_dict)
      inputDia.setFilterVisible(False)
      inputDia.setWindowTitle(''.join(['Enter the argv for ', req.request.launchfile]))
      if inputDia.exec_():
        params = inputDia.getKeywords()
        argv = []
        for p,v in params.items():
          if v:
            argv.append(''.join([p, ':=', v]))
        res = argv
        pt = ProgressThread(ident, descr, req.method, (req.args+(argv,)))
        pt.finished_signal.connect(self._progress_thread_finished)
        pt.error_signal.connect(self._progress_thread_error)
        pt.request_interact_signal.connect(self._on_request_interact)
        pt.start()
      else:
        self._progress_thread_finished(ident)
        return



class ProgressThread(QtCore.QObject, threading.Thread):
  '''
  A thread to execute a method in a thread.
  '''
  finished_signal = QtCore.Signal(str)
  '''
  @ivar: finished_signal is a signal, which is emitted, if the thread is finished.
  '''

  error_signal = QtCore.Signal(str, str, str, str)
  '''
  @ivar: error_signal is a signal (id, title, error message, detailed error message), which is emitted, 
  if an error while run of the thread was occurred.
  '''

  request_interact_signal = QtCore.Signal(str, str, InteractionNeededError)

  def __init__(self, ident, descr='', target=None, args=()):
    QtCore.QObject.__init__(self)
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
      if not self._target is None:
        #'print "PG call "
        #'print "  .. ", self._target
        #print "  -- args:", self._args
        if 'pqid' in self._target.func_code.co_varnames:
          self._target(*self._args, pqid=self._id)
        else:
          self._target(*self._args)
        #print "PG call finished"
        self.finished_signal.emit(self._id)
      else:
        self.error_signal.emit(self._id, 'No target specified')
    except InteractionNeededError as e:
      self.request_interact_signal.emit(self._id, self.descr, e)
    except DetailedError as e:
      self.error_signal.emit(self._id, e.title, e.value, e.detailed_text)
    except:
      import traceback
#      print traceback.print_exc()
      formatted_lines = traceback.format_exc(1).splitlines()
      last_line = formatted_lines[-1]
      index = 1
      while not last_line and len(formatted_lines) > index:
        index += 1
        last_line = formatted_lines[-index]
      rospy.logwarn("%s failed:\n\t%s", str(self.descr), last_line)
      self.error_signal.emit(self._id, 'Progress Job Error', str(self.descr)+" failed:\n"+last_line, traceback.format_exc(3))
