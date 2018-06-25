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
import node_manager_fkie as nm

try:
    from python_qt_binding.QtGui import QFrame, QLabel
except Exception:
    from python_qt_binding.QtWidgets import QFrame, QLabel


class MessageData(object):
    ''' '''

    def __init__(self, data):
        self.data = data

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
            if questionid == MessageFrame.TYPE_NOSCREEN:
                if self._queue[questionid]:
                    self._queue[questionid][0][1].data.append(data.data)
                else:
                    self._queue[questionid].append(('', data))
            else:
                for txt, dt in self._queue[questionid]:
                    if txt == text and dt == data:
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
        self.frameui.scrollArea.setVisible(False)
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
        if color is not None:
            bg_style = "QFrame#questionFame { background-color: %s;}" % color.name()
            self.frameui.setStyleSheet("%s" % (bg_style))
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
        if self.questionid == self.TYPE_INVALID:
            self._new_request = self._read_next_item()
            self._frameui_4_request(self._new_request)
            if self.questionid == self.TYPE_NODELET:
                self.frameui.checkBox_dnaa.setText("don't %s again, never!" % self._ask)
            else:
                self.frameui.checkBox_dnaa.setText("don't %s again, for session" % self._ask)

    def add_info_no_screen(self, nodename):
        if self.is_do_not_ask(self.TYPE_NOSCREEN):
            return
        if self.questionid == self.TYPE_NOSCREEN:
            self.data.data.append(nodename)
            self.frameui.scrollAreaLayout.addWidget(QLabel(nodename))
        else:
            self._queue.add(self.TYPE_NOSCREEN, '', MessageData([nodename]))
            if self.questionid == self.TYPE_INVALID:
                self._new_request = self._read_next_item()
                self._frameui_4_request(self._new_request)

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
            self._clear_scroll_area()
            self._new_request = self._read_next_item()
            self._frameui_4_request(self._new_request)

    def _frameui_4_request(self, request):
        if request:
            self.frameui.checkBox_dnaa.setChecked(False)
            self.frameui.setVisible(True)
            self.frameui.scrollArea.setVisible(self.frameui.scrollAreaLayout.count() > 0)
        else:
            self.questionid = 0
            self.frameui.setVisible(False)
            self.frameui.scrollArea.setVisible(False)

    def _on_question_ok(self):
        if self.frameui.checkBox_dnaa.isChecked():
            # save the decision
            self._do_not_ask[self.questionid] = True
        self._new_request = False
        self.frameui.setVisible(False)
        self.accept_signal.emit(self.questionid, self.data)
        self.questionid = 0
        self._clear_scroll_area()
        self._new_request = self._read_next_item()
        self._frameui_4_request(self._new_request)

    def _on_question_cancel(self):
        if self.frameui.checkBox_dnaa.isChecked():
            if self.questionid == self.QuestionNodelet:
                nm.settings().check_for_nodelets_at_start = False
            else:
                # save the decision
                self._do_not_ask[self.questionid] = False
        self._new_request = False
        self.frameui.setVisible(False)
        self.cancel_signal.emit(self.questionid, self.data)
        self.questionid = 0
        self._clear_scroll_area()
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
        if qid == self.TYPE_NOSCREEN:
            self.questionid = self.TYPE_NOSCREEN
            self.text = 'No screens found! See log for details!<br>The following nodes are affected:'
            self.data = data
            self.frameui.questionIcon.setPixmap(self.IMAGES[qid])
            self.frameui.questionLabel.setText(self.text)
            for nodename in data.data:
                self.frameui.scrollAreaLayout.addWidget(QLabel(nodename))
            self.frameui.scrollArea.setVisible(True)
        elif qid != self.TYPE_INVALID:
            self.questionid = qid
            self.text = text
            self.data = data
            self.frameui.questionIcon.setPixmap(self.IMAGES[qid])
            self.frameui.questionLabel.setText(text)
        return qid != self.TYPE_INVALID

    def _on_checkbox_state_changed(self, state):
        if state:
            if self.questionid == self.TYPE_NODELET:
                self.frameui.questionOkButton.setVisible(False)
        else:
            self.frameui.questionOkButton.setVisible(True)

    def _clear_scroll_area(self):
        child = self.frameui.scrollAreaLayout.takeAt(0)
        while child:
            child.widget().setParent(None)
            del child
            child = self.frameui.scrollAreaLayout.takeAt(0)
