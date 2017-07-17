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

from datetime import datetime
from python_qt_binding.QtCore import Qt, QUrl, QTimer, Signal
from python_qt_binding.QtGui import QIcon, QTextDocument
try:
    from python_qt_binding.QtGui import QApplication, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy
    from python_qt_binding.QtGui import QCheckBox, QComboBox, QDialog, QLabel, QTextBrowser, QToolButton, QWidget
except:
    from python_qt_binding.QtWidgets import QApplication, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy
    from python_qt_binding.QtWidgets import QCheckBox, QComboBox, QDialog, QLabel, QTextBrowser, QToolButton, QWidget

import math
import threading
import time

from roslib import message
import rospy

import gui_resources
import node_manager_fkie as nm


# import roslib
class EchoDialog(QDialog):

    MESSAGE_CHARS_LIMIT = 1000
    MESSAGE_LINE_LIMIT = 128
    MESSAGE_HZ_LIMIT = 10
    MAX_DISPLAY_MSGS = 25
    STATISTIC_QUEUE_LEN = 1000
    SHOW_BYTES = True
    SHOW_JITTER = True
    SHOW_STD_DEV = False
    SHOW_WINDOW_SIZE = False

    '''
  This dialog shows the output of a topic.
  '''

    finished_signal = Signal(str)
    '''
  finished_signal has as parameter the name of the topic and is emitted, if this
  dialog was closed.
  '''

    msg_signal = Signal(object, bool)
    '''
  msg_signal is a signal, which is emitted, if a new message was received.
  '''

    text_hz_signal = Signal(str)
    text_signal = Signal(str)
    '''
  text_signal is a signal, which is emitted, if a new text to display was received.
  '''

    text_error_signal = Signal(str)
    '''
  text_error_signal is a signal, which is emitted, if a new error text to display was received.
  '''

    request_pw = Signal(object)

    def __init__(self, topic, msg_type, show_only_rate=False, masteruri=None, use_ssh=False, parent=None):
        '''
        Creates an input dialog.
        @param topic: the name of the topic
        @type topic: C{str}
        @param msg_type: the type of the topic
        @type msg_type: C{str}
        @raise Exception: if no topic class was found for the given type
        '''
        QDialog.__init__(self, parent=parent)
        self._masteruri = masteruri
        masteruri_str = '' if masteruri is None else '[%s]' % masteruri
        self.setObjectName(' - '.join(['EchoDialog', topic, masteruri_str]))
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setWindowFlags(Qt.Window)
        self.setWindowTitle('%s %s %s' % ('Echo --- ' if not show_only_rate else 'Hz --- ', topic, masteruri_str))
        self.resize(728, 512)
        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(1, 1, 1, 1)
        self.mIcon = QIcon(":/icons/crystal_clear_prop_run_echo.png")
        self.setWindowIcon(self.mIcon)

        self.topic = topic
        self.show_only_rate = show_only_rate
        self.lock = threading.RLock()
        self.last_printed_count = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.times = []
        self.bytes = []

        self.message_count = 0
        self._state_message = ''
        self._scrapped_msgs = 0
        self._scrapped_msgs_sl = 0

        self._last_received_ts = 0
        self.chars_limit = self.MESSAGE_CHARS_LIMIT
        self.receiving_hz = self.MESSAGE_HZ_LIMIT
        self.line_limit = self.MESSAGE_LINE_LIMIT

        self.field_filter_fn = None
        self._latched = False
        self._msgs = []

        options = QWidget(self)
        if not show_only_rate:
            hLayout = QHBoxLayout(options)
            hLayout.setContentsMargins(1, 1, 1, 1)
            filter_string_label = QLabel('Strings:', self)
            hLayout.addWidget(filter_string_label)
            self.no_str_checkbox = no_str_checkbox = QCheckBox('hide')
            no_str_checkbox.toggled.connect(self.on_no_str_checkbox_toggled)
            hLayout.addWidget(no_str_checkbox)
            self.combobox_reduce_ch = QComboBox(self)
            self.combobox_reduce_ch.addItems([str(self.MESSAGE_LINE_LIMIT), '0', '80', '256', '1024'])
            self.combobox_reduce_ch.activated[str].connect(self.combobox_reduce_ch_activated)
            self.combobox_reduce_ch.setEditable(True)
            self.combobox_reduce_ch.setToolTip("Set maximum line width. 0 disables the limit.")
            hLayout.addWidget(self.combobox_reduce_ch)
            filter_array_label = QLabel('   Arrays:', self)
            hLayout.addWidget(filter_array_label)
            self.no_arr_checkbox = no_arr_checkbox = QCheckBox('hide')
            no_arr_checkbox.toggled.connect(self.on_no_arr_checkbox_toggled)
            hLayout.addWidget(no_arr_checkbox)
#      reduce_ch_label = QLabel('ch', self)
#      hLayout.addWidget(reduce_ch_label)
            # add spacer
            spacerItem = QSpacerItem(515, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)
            hLayout.addItem(spacerItem)
            filter_msg_label = QLabel('Msg:', self)
            hLayout.addWidget(filter_msg_label)
            # add combobox for displaying frequency of messages
            self.combobox_chars_hz = QComboBox(self)
            self.combobox_chars_hz.addItems([str(self.MESSAGE_CHARS_LIMIT), '0', '500', '1000', '5000'])
            self.combobox_chars_hz.activated[str].connect(self.on_combobox_chars_activated)
            self.combobox_chars_hz.setEditable(True)
            self.combobox_chars_hz.setToolTip("Set maximum displayed chars of a message. 0 disables the limit.")
            hLayout.addWidget(self.combobox_chars_hz)
            displ_chars_label = QLabel('chars', self)
            hLayout.addWidget(displ_chars_label)
            # add combobox for displaying frequency of messages
            self.combobox_displ_hz = QComboBox(self)
            self.combobox_displ_hz.addItems([str(self.MESSAGE_HZ_LIMIT), '0', '0.1', '1', '50', '100', '1000'])
            self.combobox_displ_hz.activated[str].connect(self.on_combobox_hz_activated)
            self.combobox_displ_hz.setEditable(True)
            self.combobox_displ_hz.setToolTip("Set maximum displayed message rate in Hz. 0 disables the limit.")
            hLayout.addWidget(self.combobox_displ_hz)
            displ_hz_label = QLabel('Hz', self)
            hLayout.addWidget(displ_hz_label)
            # add combobox for count of displayed messages
            self.combobox_msgs_count = QComboBox(self)
            self.combobox_msgs_count.addItems([str(self.MAX_DISPLAY_MSGS), '0', '50', '100'])
            self.combobox_msgs_count.activated[str].connect(self.on_combobox_count_activated)
            self.combobox_msgs_count.setEditable(True)
            self.combobox_msgs_count.setToolTip("Set maximum displayed message count. 0 disables the limit.")
            hLayout.addWidget(self.combobox_msgs_count)
            displ_count_label = QLabel('#', self)
            hLayout.addWidget(displ_count_label)
            # add topic control button for unsubscribe and subscribe
            self.topic_control_button = QToolButton(self)
            self.topic_control_button.setText('stop')
            self.topic_control_button.setIcon(QIcon(':/icons/deleket_deviantart_stop.png'))
            self.topic_control_button.clicked.connect(self.on_topic_control_btn_clicked)
            hLayout.addWidget(self.topic_control_button)
            # add clear button
            clearButton = QToolButton(self)
            clearButton.setText('clear')
            clearButton.clicked.connect(self.on_clear_btn_clicked)
            hLayout.addWidget(clearButton)
            self.verticalLayout.addWidget(options)

        self.display = QTextBrowser(self)
        self.display.setReadOnly(True)
        self.verticalLayout.addWidget(self.display)
        self.display.document().setMaximumBlockCount(500)
        self.max_displayed_msgs = self.MAX_DISPLAY_MSGS
        self._blocks_in_msg = None
        self.display.setOpenLinks(False)
        self.display.anchorClicked.connect(self._on_display_anchorClicked)

        self.status_label = QLabel('0 messages', self)
        self.verticalLayout.addWidget(self.status_label)

        # subscribe to the topic
        errmsg = ''
        try:
            self.__msg_class = message.get_message_class(msg_type)
            if not self.__msg_class:
                errmsg = "Cannot load message class for [%s]. Did you build messages?" % msg_type
#        raise Exception("Cannot load message class for [%s]. Did you build messages?"%msg_type)
        except Exception as e:
            self.__msg_class = None
            errmsg = "Cannot load message class for [%s]. Did you build messagest?\nError: %s" % (msg_type, e)
#      raise Exception("Cannot load message class for [%s]. Did you build messagest?\nError: %s"%(msg_type, e))
        # variables for Subscriber
        self.msg_signal.connect(self._append_message)
        self.sub = None

        # vairables for SSH connection
        self.ssh_output_file = None
        self.ssh_error_file = None
        self.ssh_input_file = None
        self.text_signal.connect(self._append_text)
        self.text_hz_signal.connect(self._append_text_hz)
        self._current_msg = ''
        self._current_errmsg = ''
        self.text_error_signal.connect(self._append_error_text)

        # decide, which connection to open
        if use_ssh:
            self.__msg_class = None
            self._on_display_anchorClicked(QUrl(self._masteruri))
        elif self.__msg_class is None:
            errtxt = '<pre style="color:red; font-family:Fixedsys,Courier,monospace; padding:10px;">\n%s</pre>' % (errmsg)
            self.display.setText('<a href="%s">open using SSH</a>' % (masteruri))
            self.display.append(errtxt)
        else:
            self.sub = rospy.Subscriber(self.topic, self.__msg_class, self._msg_handle)

        self.print_hz_timer = QTimer()
        self.print_hz_timer.timeout.connect(self._on_calc_hz)
        self.print_hz_timer.start(1000)
        self._start_time = time.time()

#    print "======== create", self.objectName()
#
#  def __del__(self):
#    print "******* destroy", self.objectName()

#  def hideEvent(self, event):
#    self.close()

    def closeEvent(self, event):
        if self.sub is not None:
            self.sub.unregister()
            del self.sub
        try:
            self.ssh_output_file.close()
            self.ssh_error_file.close()
            # send Ctrl+C to remote process
            self.ssh_input_file.write('%s\n' % chr(3))
            self.ssh_input_file.close()
        except:
            pass
        self.finished_signal.emit(self.topic)
        if self.parent() is None:
            QApplication.quit()
#    else:
#      self.setParent(None)

    def create_field_filter(self, echo_nostr, echo_noarr):
        def field_filter(val):
            try:
                # fields = val.__slots__
                # field_types = val._slot_types
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

    def combobox_reduce_ch_activated(self, ch_txt):
        try:
            self.line_limit = int(ch_txt)
        except ValueError:
            try:
                self.line_limit = float(ch_txt)
            except ValueError:
                self.combobox_reduce_ch.setEditText(str(self.line_limit))
        self.display.clear()
        for msg, current_time in self._msgs:
            self._append_message(msg, self._latched, current_time, False)

    def on_combobox_chars_activated(self, chars_txt):
        try:
            self.chars_limit = int(chars_txt)
        except ValueError:
            try:
                self.chars_limit = float(chars_txt)
            except ValueError:
                self.combobox_displ_chars.setEditText(str(self.chars_limit))
        self.display.clear()
        for msg, current_time in self._msgs:
            self._append_message(msg, self._latched, current_time, False)

    def on_combobox_hz_activated(self, hz_txt):
        try:
            self.receiving_hz = int(hz_txt)
        except ValueError:
            try:
                self.receiving_hz = float(hz_txt)
            except ValueError:
                self.combobox_displ_hz.setEditText(str(self.receiving_hz))

    def on_combobox_count_activated(self, count_txt):
        try:
            self.max_displayed_msgs = int(count_txt)
            self._blocks_in_msg = None
        except ValueError:
            self.combobox_msgs_count.setEditText(str(self.max_displayed_msgs))

    def on_clear_btn_clicked(self):
        self.display.clear()
        with self.lock:
            self.message_count = 0
            self._scrapped_msgs = 0
            del self.times[:]
            del self.bytes[:]

    def on_topic_control_btn_clicked(self):
        try:
            if self.sub is None and self.ssh_output_file is None:
                if self.__msg_class:
                    self.sub = rospy.Subscriber(self.topic, self.__msg_class, self._msg_handle)
                    self._start_time = time.time()
                else:
                    self._on_display_anchorClicked(QUrl(self._masteruri))
                self.topic_control_button.setText('stop')
                self.topic_control_button.setIcon(QIcon(':/icons/deleket_deviantart_stop.png'))
            else:
                if self.sub is not None:
                    self.sub.unregister()
                    self.sub = None
                elif self.ssh_output_file is not None:
                    self.ssh_output_file.close()
                    self.ssh_error_file.close()
                    self.ssh_output_file = None
                self.topic_control_button.setText('play')
                self.topic_control_button.setIcon(QIcon(':/icons/deleket_deviantart_play.png'))
                self.no_str_checkbox.setEnabled(True)
                self.no_arr_checkbox.setEnabled(True)
        except Exception as e:
            rospy.logwarn('Error while stop/play echo for topic %s: %s' % (self.topic, e))

    def _msg_handle(self, data):
        self.msg_signal.emit(data, (data._connection_header['latching'] != '0'))

    def _append_message(self, msg, latched, current_time=None, store=True):
        '''
        Adds a label to the dialog's layout and shows the given text.
        @param msg: the text to add to the dialog
        @type msg: message object
        '''
        if current_time is None:
            current_time = time.time()
        self._latched = latched
        if store:
            self._msgs.append((msg, current_time))
            if len(self._msgs) > 25:
                self._msgs.pop()
        msg_len = -1
        if (self.SHOW_BYTES or self.show_only_rate):
            buff = None
            try:
                from cStringIO import StringIO  # Python 2.x
                buff = StringIO()
                import os
                msg.serialize(buff)
                buff.seek(0, os.SEEK_END)
                msg_len = buff.tell()
            except ImportError:
                from io import BytesIO  # Python 3.x
                buff = BytesIO()
                msg.serialize(buff)
                msg_len = buff.getbuffer().nbytes
        if store:
            self._count_messages(current_time, msg_len)
        # skip messages, if they are received often then MESSAGE_HZ_LIMIT
        if self._last_received_ts != 0 and self.receiving_hz != 0:
            if current_time - self._last_received_ts < 1.0 / self.receiving_hz:
                if (not latched or (latched and current_time - self._start_time > 3.0)):
                    self._scrapped_msgs += 1
                    self._scrapped_msgs_sl += 1
                    return
        self._last_received_ts = current_time
        if not self.show_only_rate:
            # convert message to string and reduce line width to current limit
            msg = message.strify_message(msg, field_filter=self.field_filter_fn)
            if isinstance(msg, tuple):
                msg = msg[0]
            msg = self._trim_width(msg)
            msg = msg.replace('<', '&lt;').replace('>', '&gt;')
            msg_cated = False
            if self.chars_limit != 0 and len(msg) > self.chars_limit:
                msg = msg[0:self.chars_limit]
                msg_cated = True
            # create a notification about scrapped messages
            if self._scrapped_msgs_sl > 0:
                txt = '<pre style="color:red; font-family:Fixedsys,Courier,monospace; padding:10px;">scrapped %s message because of Hz-settings</pre>' % self._scrapped_msgs_sl
                self.display.append(txt)
                self._scrapped_msgs_sl = 0
            txt = '<pre style="background-color:#FFFCCC; color:#000000;font-family:Fixedsys,Courier; padding:10px;">---------- %s --------------------\n%s</pre>' % (datetime.now().strftime("%d.%m.%Y %H:%M:%S.%f"), msg)
            # set the count of the displayed messages on receiving the first message
            self._update_max_msg_count(txt)
            self.display.append(txt)
            if msg_cated:
                txt = '<pre style="color:red; font-family:Fixedsys,Courier,monospace; padding:10px;">message has been cut off</pre>'
                self.display.append(txt)
        self._print_status()

    def _count_messages(self, ts=time.time(), msg_len=-1):
        '''
        Counts the received messages. Call this method only on receive message.
        '''
        current_time = ts
        with self.lock:
            # time reset
            if self.msg_t0 < 0 or self.msg_t0 > current_time:
                self.msg_t0 = current_time
                self.msg_tn = current_time
                self.times = []
                self.bytes = []
            else:
                self.times.append(current_time - self.msg_tn)
                if msg_len > -1:
                    self.bytes.append(msg_len)
                self.msg_tn = current_time
            # keep only statistics for the last 5000 messages so as not to run out of memory
            if len(self.times) > self.STATISTIC_QUEUE_LEN:
                self.times.pop(0)
            if len(self.bytes) > self.STATISTIC_QUEUE_LEN:
                self.bytes.pop(0)
            self.message_count += 1

    def _trim_width(self, msg):
        '''
        reduce line width to current limit
        :param msg: the message
        :type msg: str
        :return: trimmed message
        '''
        result = msg
        if self.line_limit != 0:
            a = ''
            for l in msg.splitlines():
                a = a + (l if len(l) <= self.line_limit else l[0:self.line_limit - 3] + '...') + '\n'
            result = a
        return result

    def _update_max_msg_count(self, txt):
        '''
        set the count of the displayed messages on receiving the first message
        :param txt: text of the message, which will be added to the document
        :type txt: str
        '''
        if self._blocks_in_msg is None:
            td = QTextDocument(txt)
            self._blocks_in_msg = td.blockCount()
            self.display.document().setMaximumBlockCount(self._blocks_in_msg * self.max_displayed_msgs)

    def _on_calc_hz(self):
        if rospy.is_shutdown():
            self.close()
            return
        if not self.show_only_rate and time.time() - self._last_received_ts > 1:
            # create a notification about scrapped messages
            if self._scrapped_msgs_sl > 0:
                txt = '<pre style="color:red; font-family:Fixedsys,Courier,monospace; padding:10px;">scrapped %s message because of Hz-settings</pre>' % self._scrapped_msgs_sl
                self._scrapped_msgs_sl = 0
                self.display.append(txt)
        if self.message_count == self.last_printed_count:
            return
        with self.lock:
            message_rate = ''
            message_bytes = ''
            message_jitter = ''
            message_window = ''
            message_std_dev = ''
            message_scrapped = ''
            sum_times = sum(self.times)
            if sum_times == 0:
                sum_times = 1
            if (self.SHOW_BYTES or self.show_only_rate) and self.bytes:
                sum_bytes = sum(self.bytes)
                avg = sum_bytes / len(self.bytes)
                last = self.bytes[-1]
                if avg != last:
                    message_bytes = "size[ last: %s, avg: %s ]" % (self._normilize_size_print(last), self._normilize_size_print(avg))
                else:
                    message_bytes = "size: %s" % (self._normilize_size_print(last))
                byte_rate = float(sum_bytes) / float(sum_times)
                message_bytes += " bw: %s" % (self._normilize_size_print(byte_rate))
            # the code from ROS rostopic
            n = len(self.times)
            if n < 2:
                return
            mean = sum_times / n
            rate = 1. / mean if mean > 0. else 0
            message_rate = "average rate: %.3f" % rate
            # min and max
            if self.SHOW_JITTER or self.show_only_rate:
                max_delta = max(self.times)
                min_delta = min(self.times)
                message_jitter = "jitter[ min: %.3fs   max: %.3fs ]" % (min_delta, max_delta)
            # std dev
            self.last_printed_count = self.message_count
            if self.SHOW_STD_DEV or self.show_only_rate:
                std_dev = math.sqrt(sum((x - mean) ** 2 for x in self.times) / n)
                message_std_dev = "std dev: %.5fs" % (std_dev)
            if self.SHOW_WINDOW_SIZE or self.show_only_rate:
                message_window = "window: %s" % (n + 1)
            if self._scrapped_msgs > 0:
                message_scrapped += "scrapped msgs: %s" % self._scrapped_msgs
            self._state_message = ''
            for msg in [message_rate, message_jitter, message_bytes, message_std_dev, message_window, message_scrapped]:
                if msg:
                    if self._state_message:
                        self._state_message += '    '
                    self._state_message += msg
            self._print_status()
            if self.show_only_rate:
                self.display.append(self._state_message)

    def _normilize_size_print(self, size):
        if size > 999999:
            return "%.2fMB" % (size / 1048576.0)
        if size > 999:
            return "%.2fKB" % (size / 1024.0)
        return "%dB" % size

    def _print_status(self):
        text = '%s messages    %s' % (self.message_count, self._state_message)
        if self._latched:
            text = "[latched] %s" % text
        self.status_label.setText(text)

    def _append_text(self, text):
        '''
        Append echo text received through the SSH.
        '''
        with self.lock:
            self._current_msg += text
            if self._current_msg.find('---') != -1:
                messages = self._current_msg.split('---')
                for m in messages[:-1]:
                    current_time = time.time()
                    self._count_messages(current_time)
                    # limit the displayed text width
                    m = self._trim_width(m)
                    txt = '<pre style="background-color:#FFFCCC; color:#000000;font-family:Fixedsys,Courier; padding:10px;">---------- %s --------------------\n%s</pre>' % (datetime.now().strftime("%d.%m.%Y %H:%M:%S.%f"), m)
                    # set the count of the displayed messages on receiving the first message
                    self._update_max_msg_count(txt)
                    self.display.append(txt)
                self._current_msg = messages[-1]
            self._print_status()

    def _append_error_text(self, text):
        '''
        Append error text received through the SSH.
        '''
        with self.lock:
            self._current_errmsg += text
            if self._current_errmsg.find('\n') != -1:
                messages = self._current_errmsg.split('\n')
                for m in messages[:-1]:
                    txt = '<pre style="color:red; font-family:Fixedsys,Courier,monospace; padding:10px;">%s</pre>' % m
                    self.display.append(txt)
                self._current_errmsg = messages[-1]

    def _append_text_hz(self, text):
        '''
        Append text received through the SSH for hz view.
        '''
        with self.lock:
            self._current_msg += text
            if self._current_msg.find('\n') != -1:
                messages = self._current_msg.split('\n')
                for m in messages[:-1]:
                    txt = '<div style="font-family:Fixedsys,Courier;">%s</div>' % (m)
                    self.display.append(txt)
                self._current_msg = messages[-1]

    def _on_display_anchorClicked(self, url, user=None, pw=None):
        try:
            ok = False
            if self.show_only_rate:
                self.ssh_input_file, self.ssh_output_file, self.ssh_error_file, ok = nm.ssh().ssh_exec(url.host(), ['rostopic hz %s' % (self.topic)], user, pw, auto_pw_request=True, get_pty=True)
                self.status_label.setText('connected to %s over SSH' % url.host())
            else:
                self.combobox_displ_hz.setEnabled(False)
                nostr = '--nostr' if self.no_str_checkbox.isChecked() else ''
                noarr = '--noarr' if self.no_arr_checkbox.isChecked() else ''
                self.ssh_input_file, self.ssh_output_file, self.ssh_error_file, ok = nm.ssh().ssh_exec(url.host(), ['rostopic echo %s %s %s' % (nostr, noarr, self.topic)], user, pw, auto_pw_request=True, get_pty=True)
            if ok:
                self.display.clear()
                target = self._read_output_hz if self.show_only_rate else self._read_output
                thread = threading.Thread(target=target, args=((self.ssh_output_file,)))
                thread.setDaemon(True)
                thread.start()
                thread = threading.Thread(target=self._read_error, args=((self.ssh_error_file,)))
                thread.setDaemon(True)
                thread.start()
            elif self.ssh_output_file:
                self.ssh_output_file.close()
                self.ssh_error_file.close()
        except Exception as e:
            self._append_error_text('%s\n' % e)
#      import traceback
#      print traceback.format_exc()

    def _read_output_hz(self, output_file):
        try:
            while not output_file.closed:
                text = output_file.read(1)
                if text:
                    self.text_hz_signal.emit(text)
        except:
            pass
#      import traceback
#      print traceback.format_exc()

    def _read_output(self, output_file):
        while not output_file.closed:
            text = output_file.read(1)
            if text:
                self.text_signal.emit(text)

    def _read_error(self, error_file):
        try:
            while not error_file.closed:
                text = error_file.read(1)
                if text:
                    self.text_error_signal.emit(text)
        except:
            pass
#      import traceback
#      print traceback.format_exc()
