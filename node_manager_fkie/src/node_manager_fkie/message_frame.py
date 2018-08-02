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
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from node_manager_fkie.common import utf8
from node_manager_fkie.html_delegate import HTMLDelegate
import node_manager_fkie as nm

try:
    from python_qt_binding.QtGui import QFrame, QLabel
except Exception:
    from python_qt_binding.QtWidgets import QFrame, QLabel


class MessageData(object):
    ''' '''

    def __init__(self, data, data_list=[]):
        self.data = data
        self.data_list = data_list

    def __str__(self):
        return utf8(self.data)

    def __eq__(self, other):
        return self.data == other.data

    def __ne__(self, other):
        return self.data != other.data


class MessageQueue(object):

    def __init__(self):
        self._queue = {
            MessageFrame.TYPE_EMPTY: [],
            MessageFrame.TYPE_QUESTION: [],
            MessageFrame.TYPE_LAUNCH_FILE: [],
            MessageFrame.TYPE_DEFAULT_CFG: [],
            MessageFrame.TYPE_NODELET: [],
            MessageFrame.TYPE_TRANSFER: [],
            MessageFrame.TYPE_BINARY: [],
            MessageFrame.TYPE_NOSCREEN: []
        }

    def add(self, questionid, text, data):
        if questionid in self._queue.keys():
            for txt, dt in self._queue[questionid]:
                if txt == text and dt == data:
                    no_in_list = [x for x in data.data_list if x not in dt.data_list]
                    for item in no_in_list:
                        dt.data_list.append(item)
                    return
            self._queue[questionid].append((text, data))

    def get(self):
        '''
        Returns a tuple of (questionid, text, data)
        '''
        for qid, values in self._queue.iteritems():
            if values:
                text, data = values.pop(0)
                return (qid, text, data)
        return (0, '', None)


class MessageFrame(QFrame):

    accept_signal = Signal(int, MessageData)
    ''' @ivar: A signal on accept button clicked (id, data)'''

    cancel_signal = Signal(int, MessageData)
    ''' @ivar: A signal on cancel button clicked (id, data)'''

    TYPE_INVALID = 0
    TYPE_EMPTY = 1
    TYPE_QUESTION = 2
    TYPE_LAUNCH_FILE = 3
    TYPE_DEFAULT_CFG = 4
    TYPE_NODELET = 5
    TYPE_TRANSFER = 6
    TYPE_BINARY = 7
    TYPE_NOSCREEN = 8

    ICON_SIZE = 32

    def __init__(self, parent=None, info=False):
        QFrame.__init__(self, parent=parent)
        self.setObjectName('MessageFrame')
        self.questionid = self.TYPE_INVALID
        self.text = ""
        self.data = MessageData(None)
        self.IMAGES = {1: QPixmap(),
                       2: QPixmap(':/icons/crystal_clear_question.png').scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       3: QPixmap(':/icons/crystal_clear_launch_file.png').scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       4: QPixmap(":/icons/default_cfg.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       5: QPixmap(":/icons/crystal_clear_nodelet_q.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       6: QPixmap(":/icons/crystal_clear_launch_file_transfer.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       7: QPixmap(":/icons/crystal_clear_question.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       8: QPixmap(":/icons/crystal_clear_no_io.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
                       }
        self._new_request = False
        self.frameui = QFrame()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MessageFrame.ui')
        loadUi(ui_file, self.frameui)
        color = QColor(255, 207, 121)
        self.frameui.setVisible(False)
        self.frameui.listLabel.setVisible(False)
        self.frameui.questionOkButton.clicked.connect(self._on_question_ok)
        self.frameui.questionCancelButton.clicked.connect(self._on_question_cancel)
        self.frameui.checkBox_dnaa.stateChanged.connect(self._on_checkbox_state_changed)
        self._ask = 'ask'
        if info:
            color = QColor(232, 104, 80)
            self.frameui.questionCancelButton.setVisible(False)
            self._ask = 'show'
        bg_style = "QFrame#questionFame { background-color: %s;}" % color.name()
        self.frameui.setStyleSheet("%s" % (bg_style))
        self._queue = MessageQueue()
        self._do_not_ask = {}

    def show_question(self, questionid, text, data=MessageData(None), color=None):
        if questionid == 0:
            return
        try:
            # is it in the list for not ask again?
            if self._do_not_ask[questionid]:
                self.accept_signal.emit(questionid, data)
            else:
                self.cancel_signal.emit(questionid, data)
            return
        except Exception:
            pass
        if self.questionid != questionid or self.text != text or data != self.data:
            self._queue.add(questionid, text, data)
        elif data.data_list:  # same question again
            # update the list of files or nodes which causes this question in current question
            for dt in data.data_list:
                if dt not in self.data.data_list:
                    self.data.data_list.append(dt)
            self._update_list_label(self.data.data_list)
        # if no question is active pop first element from the queue
        if self.questionid == self.TYPE_INVALID:
            self._new_request = self._read_next_item()
            self._frameui_4_request(self._new_request)
            if self.questionid in [self.TYPE_NODELET, self.TYPE_NOSCREEN]:
                self.frameui.checkBox_dnaa.setText("don't %s again, never!" % self._ask)
            else:
                self.frameui.checkBox_dnaa.setText("don't %s again, for session" % self._ask)

    def show_info(self, infoid, text, data=MessageData(None), color=None):
        self.show_question(infoid, text=text, data=data, color=color)

    def is_do_not_ask(self, questionid):
        try:
            # is it in the list for not ask again?
            return self._do_not_ask[questionid]
        except Exception:
            return False

    def hide_question(self, questionids):
        if self.questionid in questionids:
            self._new_request = False
            self.frameui.setVisible(False)
            self.cancel_signal.emit(self.questionid, self.data)
            self.questionid = 0
            self._update_list_label([])
            self._new_request = self._read_next_item()
            self._frameui_4_request(self._new_request)

    def _update_list_label(self, items=[]):
        '''
        Put list elements into the list label in the question frame
        '''
        if items:
            self.frameui.listLabel.setText('')
            for item in items:
                ltext = self.frameui.listLabel.text()
                if ltext:
                    self.frameui.listLabel.setText("%s, %s" % (ltext, HTMLDelegate.toHTML(item)))
                else:
                    self.frameui.listLabel.setText("%s" % (HTMLDelegate.toHTML(item)))
            self.frameui.listLabel.setVisible(True)
        else:
            self.frameui.listLabel.setText('')
            self.frameui.listLabel.setVisible(False)

    def _frameui_4_request(self, request):
        if request:
            self.frameui.checkBox_dnaa.setChecked(False)
            self.frameui.setVisible(True)
            self.frameui.listLabel.setVisible(True)
        else:
            self.questionid = 0
            self.frameui.setVisible(False)
            self.frameui.listLabel.setVisible(False)

    def _on_question_ok(self):
        self._new_request = False
        self.frameui.setVisible(False)
        self.accept_signal.emit(self.questionid, self.data)
        self.questionid = 0
        self._update_list_label([])
        self._new_request = self._read_next_item()
        self._frameui_4_request(self._new_request)

    def _on_question_cancel(self):
        self._new_request = False
        self.frameui.setVisible(False)
        self.cancel_signal.emit(self.questionid, self.data)
        self.questionid = 0
        self._update_list_label([])
        self._new_request = self._read_next_item()
        self._frameui_4_request(self._new_request)

    def _is_launch_data_in_queue(self, newdata):
        for _, data, _ in self._queue_launchfile:
            if data == newdata:
                return True
        return False

    def _is_transfer_data_in_queue(self, newdata):
        for _, data, _ in self._queue_transfer_files:
            if data == newdata:
                return True
        return False

    def _is_other_data_in_queue(self, questionid, text, data):
        for cqid, ctxt, cd, _ in self._queue_other:
            if cqid == questionid and cd == data and ctxt == text:
                return True
        return False

    def _read_next_item(self):
        (qid, text, data) = self._queue.get()
        if qid != self.TYPE_INVALID:
            self.questionid = qid
            self.text = text
            self.data = data
            self.frameui.questionIcon.setPixmap(self.IMAGES[qid])
            self.frameui.questionLabel.setText(text)
            self._update_list_label(self.data.data_list)
        return qid != self.TYPE_INVALID

    def _on_checkbox_state_changed(self, state):
        if self.questionid == self.TYPE_NODELET:
            self.frameui.questionOkButton.setVisible(not state)
            nm.settings().check_for_nodelets_at_start = not state
        elif self.questionid == self.TYPE_NOSCREEN:
            self.frameui.questionCancelButton.setVisible(not state)
            nm.settings().show_noscreen_error = not state
        else:
            self._do_not_ask[self.questionid] = state

    def _clear_scroll_area(self):
        child = self.frameui.scrollAreaLayout.takeAt(0)
        while child:
            child.widget().setParent(None)
            del child
            child = self.frameui.scrollAreaLayout.takeAt(0)
