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
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QPixmap
from node_manager_fkie.common import utf8


try:
    from python_qt_binding.QtGui import QFrame
except:
    from python_qt_binding.QtWidgets import QFrame


class MessageData(object):
    ''' '''

    def __init__(self, data):
        self.data = data


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

    QuestionInvalid = 0
    QuestionNoname = 1
    QuestionLaunchFile = 2
    QuestionDefaultCfg = 3
    QuestionNodelet = 4
    QuestionBinary = 5

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
                       5: QPixmap(":/icons/crystal_clear_question.png").scaled(self.ICON_SIZE, self.ICON_SIZE, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
                       }
        self._new_request = False
        self.frameui = QFrame()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MessageFrame.ui')
        loadUi(ui_file, self.frameui)
        self.frameui.setVisible(False)
        self.frameui.questionOkButton.clicked.connect(self._on_question_ok)
        self.frameui.questionCancelButton.clicked.connect(self._on_question_cancel)

    def show_question(self, questionid, text, data=MessageData(None), icon=0):
        self._new_request = True
        self.questionid = questionid
        self.text = text
        self.iconid = icon
        self.data = data
        self.frameui.questionIcon.setPixmap(self.IMAGES[icon])
        self.frameui.questionLabel.setText(text)
        self.frameui.setVisible(True)

    def hide_question(self):
        self._new_request = False
        self.frameui.setVisible(False)
        self.cancel_signal.emit(self.questionid, self.data)
        if not self._new_request:
            self.questionid = 0

    def _on_question_ok(self):
        self._new_request = False
        self.frameui.setVisible(False)
        self.accept_signal.emit(self.questionid, self.data)
        if not self._new_request:
            self.questionid = 0

    def _on_question_cancel(self):
        self._new_request = False
        self.frameui.setVisible(False)
        self.cancel_signal.emit(self.questionid, self.data)
        if not self._new_request:
            self.questionid = 0


