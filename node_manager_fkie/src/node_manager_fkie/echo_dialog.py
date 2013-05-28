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


from python_qt_binding import QtCore, QtGui

import time
import math

import roslib
import roslib.message
import rospy
import threading

class EchoDialog(QtGui.QDialog):
  
  MESSAGE_HZ_LIMIT = 100

  '''
  This dialog shows the output of a topic.
  '''
  
  finished_signal = QtCore.Signal(str)
  '''
  finished_signal has as parameter the name of the topic and is emitted, if this
  dialog was closed.
  '''
  
  msg_signal = QtCore.Signal(str)
  '''
  msg_signal is a signal, which is emitted, if a new message was received.
  '''
  
  def __init__(self, topic, type, show_only_rate=False, masteruri=None, parent=None):
    '''
    Creates an input dialog.
    @param topic: the name of the topic
    @type topic: C{str}
    @param type: the type of the topic
    @type type: C{str}
    @raise Exception: if no topic class was found for the given type
    '''
    QtGui.QDialog.__init__(self, parent=parent)
    masteruri_str = '' if masteruri is None else ''.join([' [', str(masteruri), ']'])
    self.setObjectName(' - '.join(['EchoDialog', topic, masteruri_str]))
    self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
    self.setWindowFlags(QtCore.Qt.Window)
    self.setWindowTitle(''.join(['Echo of ' if not show_only_rate else 'Hz of ', topic, masteruri_str]))
    self.resize(728,512)
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.verticalLayout.setContentsMargins(1, 1, 1, 1)
    self.mIcon = QtGui.QIcon(":/icons/crystal_clear_prop_run_echo.png")
    self.setWindowIcon(self.mIcon)
        
    self.topic = topic
    self.show_only_rate = show_only_rate
    self.lock = threading.Lock()
    self.last_printed_tn = 0
    self.msg_t0 = -1.
    self.msg_tn = 0
    self.times =[]
        
    self.message_count = 0

    self._rate_message = ''

    self.message_ignored_count = 0
    self.ts_first_msg = 0
    self.message_interval_count = 0
    self.message_interval_count_last = 0
    
    self.field_filter_fn = None
    
    options = QtGui.QWidget(self)
    if not show_only_rate:
      hLayout = QtGui.QHBoxLayout(options)
      hLayout.setContentsMargins(1, 1, 1, 1)
      self.no_str_checkbox = no_str_checkbox = QtGui.QCheckBox('Hide strings')
      no_str_checkbox.toggled.connect(self.on_no_str_checkbox_toggled)
      hLayout.addWidget(no_str_checkbox)
      self.no_arr_checkbox = no_arr_checkbox = QtGui.QCheckBox('Hide arrays')
      no_arr_checkbox.toggled.connect(self.on_no_arr_checkbox_toggled)
      hLayout.addWidget(no_arr_checkbox)
      # add spacer
      spacerItem = QtGui.QSpacerItem(515, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
      hLayout.addItem(spacerItem)
      # add clear button
      cancelButton = QtGui.QToolButton(self)
      cancelButton.setText('clear')
      cancelButton.clicked.connect(self.on_clear_btn_clicked)
      hLayout.addWidget(cancelButton)
      self.verticalLayout.addWidget(options)
    
    self.display = QtGui.QTextEdit(self)
    self.display.setReadOnly(True)
    self.verticalLayout.addWidget(self.display);

    self.status_label = QtGui.QLabel('0 messages', self)
    self.verticalLayout.addWidget(self.status_label)

    # subscribe to the topic
    msg_class = roslib.message.get_message_class(type)
    if not msg_class:
      raise Exception("Cannot load message class for [%s]. Are your messages built?"%type)
    self.sub = rospy.Subscriber(topic, msg_class, self._msg_handle)
    self.msg_signal.connect(self._append_message)
    
    self.print_hz_timer = QtCore.QTimer()
    self.print_hz_timer.timeout.connect(self._on_calc_hz)
    self.print_hz_timer.start(1000)

#    print "======== create", self.objectName()
#
#  def __del__(self):
#    print "******* destroy", self.objectName()

#  def hideEvent(self, event):
#    self.close()

  def closeEvent (self, event):
    self.sub.unregister()
    del self.sub
    self.finished_signal.emit(self.topic)
    if self.parent() is None:
      QtGui.QApplication.quit()
#    else:
#      self.setParent(None)
  
  def create_field_filter(self, echo_nostr, echo_noarr):
    def field_filter(val):
      try:
        fields = val.__slots__
        field_types = val._slot_types
        for f, t in zip(val.__slots__, val._slot_types):
          if echo_noarr and '[' in t:
            continue
          elif echo_nostr and 'string' in t:
            continue
          yield f
      except:
        pass
    return field_filter

  def on_no_str_checkbox_toggled(self, state):
    self.field_filter_fn = self.create_field_filter(state, self.no_arr_checkbox.isChecked())

  def on_no_arr_checkbox_toggled(self, state):
    self.field_filter_fn = self.create_field_filter(self.no_str_checkbox.isChecked(), state)

  def on_clear_btn_clicked(self):
    self.display.clear()
    
  def _msg_handle(self, data):
    self.msg_signal.emit(roslib.message.strify_message(data, field_filter=self.field_filter_fn))

  def _append_message(self, msg):
    '''
    Adds a label to the dialog's layout and shows the given text.
    @param msg: the text to add to the dialog
    @type msg: C{str}
    '''
    current_time = time.time()
    with self.lock:
      current_time = time.time()
      # time reset
      if self.msg_t0 < 0 or self.msg_t0 > current_time:
        self.msg_t0 = current_time
        self.msg_tn = current_time
        self.times = []
      else:
        self.times.append(current_time - self.msg_tn)
        self.msg_tn = current_time

      #only keep statistics for the last 5000 messages so as not to run out of memory
      if len(self.times) > 5000:
        self.times.pop(0)

    
    
    self.message_count += 1
#    if not self.ts_first_msg:
#      self.ts_first_msg = time.time()
#    elif int(current_time) - int(self.ts_first_msg) > 0:
#      self.message_interval_count_last = self.message_interval_count
#      self.message_interval_count = 0
#      self.ts_first_msg = current_time
#    self.message_interval_count += 1
#    if self.message_interval_count > self.MESSAGE_HZ_LIMIT:
#      self.message_ignored_count += 1
#    status_text = ' '.join([str(self.message_count), 'messages'])
#    # show rate of last second
#    if self.message_interval_count_last > 0:
#      status_text = ' '.join([status_text, ', rate: ', str(self.message_interval_count_last),'Hz'])
#    # show the info, what no more then the limit messages is displayed
#    if self.message_ignored_count > 0:
#      status_text = ' '.join([status_text, ', skipped: ', str(self.message_ignored_count), ' (maximum displayed: ', str(self.MESSAGE_HZ_LIMIT),'Hz)'])
#    self.status_label.setText(status_text)
#    # append message only if the limit is not reached
    if not self.show_only_rate:
      self.display.append(''.join(['<pre style="background-color:#FFFCCC; font-family:Fixedsys,Courier,monospace; padding:10px;">\n', msg,'\n</pre><hr>']))
    self._print_status()

  def _on_calc_hz(self):
    if rospy.is_shutdown():
      self.close()
      return
    if self.msg_tn == self.last_printed_tn:
      self._rate_message = 'no new messages'
      return
    with self.lock:
      # the code from ROS rostopic
      n = len(self.times)
      if n == 0:
        return
      mean = sum(self.times) / n
      rate = 1./mean if mean > 0. else 0

      #std dev
      std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) /n)
      # min and max
      max_delta = max(self.times)
      min_delta = min(self.times)

      self.last_printed_tn = self.msg_tn
      self._rate_message = "average rate: %.3f\tmin: %.3fs   max: %.3fs   std dev: %.5fs   window: %s"%(rate, min_delta, max_delta, std_dev, n+1)
      self._print_status()
      if self.show_only_rate:
        self.display.append(self._rate_message)


  def _print_status(self):
    status_text = ' '.join([str(self.message_count), 'messages', ', ' if self._rate_message else '', self._rate_message])
    self.status_label.setText(status_text)

