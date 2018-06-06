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
from python_qt_binding.QtGui import QPalette, QPixmap
from node_manager_fkie.common import utf8
import node_manager_fkie as nm

try:
    from python_qt_binding.QtGui import QFrame
except Exception:
    from python_qt_binding.QtWidgets import QFrame


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


class MessageFrame(QFrame):

    accept_signal = Signal(int, MessageData)
    ''' @ivar: A signal on accept button clicked (id, data)'''

    cancel_signal = Signal(int, MessageData)
    ''' @ivar: A signal on cancel button clicked (id, data)'''

    IconEmpty = 0
    IconQuestion = 1
    IconLaunchFile = 2
    IconDefaultCfg = 3
    IconNodelet = 4
    IconBinary = 5
    IconTransfer = 6

    QuestionInvalid = 0
    QuestionNoname = 1
    QuestionLaunchFile = 2
    QuestionDefaultCfg = 3
    QuestionNodelet = 4
    QuestionBinary = 5
    QuestionTransfer = 6

    ICON_SIZE = 32

    def __init__(self, parent=None):
        QFrame.__init__(self, parent=parent)
        self.setObjectName('MessageFrame')
        self.questionid = self.QuestionInvalid
        self.text = ""
        self.iconid = self.IconEmpty
        self.data = MessageData(None)
        self.IMAGES = {0: QPixmap(),
                       1: QPixmap(':/icons/crystal_clear_question.png').scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       2: QPixmap(':/icons/crystal_clear_launch_file.png').scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       3: QPixmap(":/icons/default_cfg.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       4: QPixmap(":/icons/crystal_clear_nodelet_q.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       5: QPixmap(":/icons/crystal_clear_question.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation),
                       6: QPixmap(":/icons/crystal_clear_launch_file_transfer.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
                       }
        self._new_request = False
        self.frameui = QFrame()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MessageFrame.ui')
        loadUi(ui_file, self.frameui)
        self.frameui.setVisible(False)
        bg_style = "QFrame#questionFame { background-color: lightGray;}"
        self.frameui.setStyleSheet("%s" % (bg_style))
        self.frameui.questionOkButton.clicked.connect(self._on_question_ok)
        self.frameui.questionCancelButton.clicked.connect(self._on_question_cancel)
        self.frameui.checkBox_dnaa.stateChanged.connect(self._on_checkbox_state_changed)
        # we use different queues for priority
        self._queue_launchfile = []
        self._queue_transfer_files = []
        self._queue_other = []
        self._do_not_ask = {}

    def show_question(self, questionid, text, data=MessageData(None), icon=0):
        ic = icon
        if ic == 0:
            ic = questionid
        try:
            # is it in the list for not ask again?
            if self._do_not_ask[utf8((questionid, str(data)))]:
                self.accept_signal.emit(questionid, data)
            else:
                self.cancel_signal.emit(questionid, data)
            return
        except Exception:
            pass
        self._new_request = True
        if self.questionid == 0:
            # currently not question active, show
            self.questionid = questionid
            self.text = text
            self.iconid = ic
            self.data = data
            self.frameui.questionIcon.setPixmap(self.IMAGES[ic])
            self.frameui.questionLabel.setText(text)
            self._frameui_4_request(self._new_request)
        else:
            # put it in the queue
            if questionid == 2:
                if not self._is_launch_data_in_queue(data) and self.data != data:
                    self._queue_launchfile.append((text, data, ic))
            elif questionid == 6:
                if not self._is_transfer_data_in_queue(data):
                    if self.data != data:
                        self._queue_transfer_files.append((text, data, ic))
            else:
                if not self._is_other_data_in_queue(questionid, text, data):
                    if questionid != self.questionid and self.text != text and self.data != data:
                        self._queue_other.append((questionid, text, data, ic))
        if self.questionid == self.QuestionNodelet:
            self.frameui.checkBox_dnaa.setText("don't ask again, never!")
        else:
            self.frameui.checkBox_dnaa.setText("don't ask again, temporary")

    def hide_question(self, questionids):
        if self.questionid in questionids:
            self._new_request = False
            self.frameui.setVisible(False)
            self.cancel_signal.emit(self.questionid, self.data)
            self._new_request = self._read_next_item()
            self._frameui_4_request(self._new_request)

    def _frameui_4_request(self, request):
        if request:
            self.frameui.checkBox_dnaa.setChecked(False)
            self.frameui.setVisible(True)
        else:
            self.questionid = 0
            self.frameui.setVisible(False)

    def _on_question_ok(self):
        if self.frameui.checkBox_dnaa.isChecked():
            # save the decision
            self._do_not_ask[utf8((self.questionid, str(self.data)))] = True
        self._new_request = False
        self.frameui.setVisible(False)
        self.accept_signal.emit(self.questionid, self.data)
        self._new_request = self._read_next_item()
        self._frameui_4_request(self._new_request)

    def _on_question_cancel(self):
        if self.frameui.checkBox_dnaa.isChecked():
            if self.questionid == self.QuestionNodelet:
                nm.settings().check_for_nodelets_at_start = False
            else:
                # save the decision
                self._do_not_ask[utf8((self.questionid, str(self.data)))] = False
        self._new_request = False
        self.frameui.setVisible(False)
        self.cancel_signal.emit(self.questionid, self.data)
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
        result = False
        if self._queue_launchfile:
            result = True
            text, data, icon = self._queue_launchfile.pop(0)
            self.questionid = self.QuestionLaunchFile
            self.text = text
            self.iconid = icon
            self.data = data
            self.frameui.questionIcon.setPixmap(self.IMAGES[icon])
            self.frameui.questionLabel.setText(text)
        elif self._queue_transfer_files:
            result = True
            text, data, icon = self._queue_transfer_files.pop(0)
            self.questionid = self.QuestionTransfer
            self.text = text
            self.iconid = icon
            self.data = data
            self.frameui.questionIcon.setPixmap(self.IMAGES[icon])
            self.frameui.questionLabel.setText(text)
        elif self._queue_other:
            result = True
            questionid, text, data, icon = self._queue_other.pop(0)
            self.questionid = questionid
            self.text = text
            self.iconid = icon
            self.data = data
            self.frameui.questionIcon.setPixmap(self.IMAGES[icon])
            self.frameui.questionLabel.setText(text)
        return result

    def _on_checkbox_state_changed(self, state):
        if state:
            if self.questionid == self.QuestionNodelet:
                self.frameui.questionOkButton.setVisible(False)
        else:
            self.frameui.questionOkButton.setVisible(True)
