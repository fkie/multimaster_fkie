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
#  * Neither the name of I Heart Engineering nor the names of its
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


from PySide import QtCore, QtGui

import time

import roslib
import roslib.message
import rospy

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
  
  def __init__(self, topic, type, parent=None):
    '''
    Creates an input dialog.
    @param topic: the name of the topic
    @type topic: C{str}
    @param type: the type of the topic
    @type type: C{str}
    @raise Exception: if no topic class was found for the given type
    '''
    QtGui.QDialog.__init__(self, parent)
    self.parent = parent
    self.setWindowFlags(QtCore.Qt.Window)
    self.setWindowTitle(''.join(['Echo of ', topic]))
    self.resize(728,512)
    self.setObjectName("EchoDialog")
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.verticalLayout.setContentsMargins(1, 1, 1, 1)
    
    self.topic = topic
    self._auto_scroll = True
    self.message_count = 0
    self.message_ignored_count = 0
    self.ts_first_msg = 0
    self.message_interval_count = 0
    self.message_interval_count_last = 0
    self.field_filter_fn = None
    
    
    options = QtGui.QWidget(self)
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
    self.verticalLayout.addWidget(options)
    
    self.display = QtGui.QTextEdit()
    self.display.setReadOnly(True)
    self.verticalLayout.addWidget(self.display);

    self.status_label = QtGui.QLabel('0 messages')
    self.verticalLayout.addWidget(self.status_label)

    self.finished.connect(self._finished)

    # subscribe to the topic
    msg_class = roslib.message.get_message_class(type)
    if not msg_class:
      raise Exception("Cannot load message class for [%s]. Are your messages built?"%type)
    self.sub = rospy.Subscriber(topic, msg_class, self._msg_handle)
    self.msg_signal.connect(self._append_message)

  def finish(self):
    pass

  def _finished(self, result):
    self.sub.unregister()
    del self.sub
    self.finished_signal.emit(self.topic)
    if self.parent is None:
      QtGui.QApplication.quit()

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

  def _msg_handle(self, data):
    self.msg_signal.emit(roslib.message.strify_message(data, field_filter=self.field_filter_fn))

  def _append_message(self, msg):
    '''
    Adds a label to the dialog's layout and shows the given text.
    @param msg: the text to add to the dialog
    @type msg: C{str}
    '''
    current_time = time.time()
    self.message_count += 1
    if not self.ts_first_msg:
      self.ts_first_msg = time.time()
    elif int(current_time) - int(self.ts_first_msg) > 0:
      self.message_interval_count_last = self.message_interval_count
      self.message_interval_count = 0
      self.ts_first_msg = current_time
    self.message_interval_count += 1
    if self.message_interval_count > self.MESSAGE_HZ_LIMIT:
      self.message_ignored_count += 1
    status_text = ' '.join([str(self.message_count), 'messages'])
    # show rate of last second
    if self.message_interval_count_last > 0:
      status_text = ' '.join([status_text, ', rate: ', str(self.message_interval_count_last),'Hz'])
    # show the info, what no more then the limit messages is displayed
    if self.message_ignored_count > 0:
      status_text = ' '.join([status_text, ', skipped: ', str(self.message_ignored_count), ' (maximum displayed: ', str(self.MESSAGE_HZ_LIMIT),'Hz)'])
    self.status_label.setText(status_text)
    # append message only if the limit is not reached
    if self.message_interval_count < self.MESSAGE_HZ_LIMIT:
      self.display.append(''.join(['<pre style="background-color:#FFFCCC; font-family:Fixedsys,Courier,monospace; padding:10px;">\n', msg,'\n</pre><hr>']))
