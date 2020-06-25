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



try:
    import queue
except ImportError:
    import Queue as queue  # python 2 compatibility
from datetime import datetime
from python_qt_binding.QtCore import Qt, Signal
try:
    from python_qt_binding.QtGui import QDialog, QLabel, QTextBrowser, QVBoxLayout
except Exception:
    from python_qt_binding.QtWidgets import QDialog, QLabel, QTextBrowser, QVBoxLayout
import threading
import time
import traceback

import rospy

from fkie_master_discovery.master_discovery import Discoverer
from fkie_master_discovery.udp import DiscoverSocket, QueueReceiveItem
import fkie_node_manager as nm
from fkie_node_manager_daemon.common import utf8


class NetworkDiscoveryDialog(QDialog, threading.Thread):

    TIMEOUT = 0.1

    display_clear_signal = Signal()
    display_append_signal = Signal(str)
    status_text_signal = Signal(str)
    network_join_request = Signal(int)

    def __init__(self, default_mcast_group, default_port, networks_count, parent=None):
        '''
        Creates an input dialog.
        @param default_port: the default discovery port
        @type default_port: C{int}
        @param networks_count: the count of discovering ports
        @type networks_count: C{int}
        '''
        QDialog.__init__(self, parent=parent)
        threading.Thread.__init__(self)
        self.default_port = default_port
        self.setObjectName('NetworkDiscoveryDialog')
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.setWindowFlags(Qt.Window)
        self.setWindowTitle('Network Discovery')
        self.resize(728, 512)
        self.verticalLayout = QVBoxLayout(self)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(1, 1, 1, 1)

        self.display = QTextBrowser(self)
        self.display.setReadOnly(True)
        self.verticalLayout.addWidget(self.display)
        self.display_clear_signal.connect(self.display.clear)
        self.display_append_signal.connect(self.display.append)
        self.display.anchorClicked.connect(self.on_anchorClicked)

        self.status_label = QLabel('0 messages', self)
        self.verticalLayout.addWidget(self.status_label)
        self.status_text_signal.connect(self.status_label.setText)
        self._msg_counts = dict()

        self._networks_count = networks_count
        self._running = True
        self._received_msgs = 0
        self._discovered = dict()
        self._hosts = dict()  # resolution for hostname and address
        self.mutex = threading.RLock()
        self.sockets = []
        with self.mutex:
            try:
                for p in range(networks_count):
                    msock = DiscoverSocket(default_port + p, default_mcast_group)
                    self.sockets.append(msock)
                    msock.settimeout(self.TIMEOUT)
            except Exception as e:
                self.display.setText(utf8(e))
        self.setDaemon(True)
        self.start()

    def on_heartbeat_received(self, msg, address, is_multicast):
        if len(msg) == 0:
            return
        force_update = False
        with self.mutex:
            try:
                hostname = self._hosts[address[0]]
            except Exception:
                self.status_text_signal.emit("resolve %s" % address[0])
                hostname = nm.nameres().hostname(utf8(address[0]), resolve=True)
                self._hosts[address[0]] = hostname
            try:
                (_version, _msg_tuple) = Discoverer.msg2masterState(msg, address)
                index = address[1] - self.default_port
                if index not in self._discovered:
                    self._discovered[index] = dict()
                self._discovered[index][address] = (hostname, time.time())
                if hostname not in self._msg_counts:
                    self._msg_counts[hostname] = 0
                self._msg_counts[hostname] += 1
                self._received_msgs += 1
                force_update = True
            except Exception:
                print(traceback.format_exc(1))
        if force_update:
            self._updateDisplay()

    def run(self):
        self.parent().masterlist_service.refresh(self.parent().getMasteruri(), False)
        while (not rospy.is_shutdown()) and self._running:
            with self.mutex:
                for msock in self.sockets:
                    received = True
                    while received:
                        try:
                            recv_item = msock.receive_queue.get(False)
                            self._received_msgs += 1
                            self.on_heartbeat_received(recv_item.msg, recv_item.sender_addr, (recv_item.via == QueueReceiveItem.MULTICAST))
                        except queue.Empty:
                            received = False
                status_text = 'received messages: %d' % (self._received_msgs)
                self.status_text_signal.emit(status_text)
#      self.parent().masterlist_service.refresh(self.parent().getMasteruri(), False)
            time.sleep(3)

    def closeEvent(self, event):
        self.stop()
        QDialog.closeEvent(self, event)

    def stop(self):
        self._running = False
        with self.mutex:
            for p in range(len(self.sockets)):
                try:
                    self.sockets[p].close()
                except Exception:
                    pass

    def _updateDisplay(self):
        self.display_clear_signal.emit()
        text = '<div style="font-family:Fixedsys,Courier,monospace; padding:10px;">\n'
        for index, addr_dict in self._discovered.items():
            text += 'Network <b>%s</b>: <a href="%s">join</a><dl>' % (utf8(index), utf8(index))
            for addr, (hostname, ts) in addr_dict.items():
                text += '<dt>%s   <b><u>%s</u></b> %s, received messages: %s</dt>\n' % (self._getTsStr(ts), utf8(hostname), utf8(addr), str(self._msg_counts[hostname]))
            text += '</dl><br>'
        text += '</div>'
        self.display_append_signal.emit(text)

    def _getTsStr(self, timestamp):
        dt = datetime.fromtimestamp(timestamp)
        diff = time.time() - timestamp
        diff_dt = datetime.fromtimestamp(diff)
        before = '0 sec'
        if (diff < 60):
            before = diff_dt.strftime('%S sec')
        elif (diff < 3600):
            before = diff_dt.strftime('%M:%S min')
        elif (diff < 86400):
            before = diff_dt.strftime('%H:%M:%S std')
        else:
            before = diff_dt.strftime('%d Day(s) %H:%M:%S')
        return '%s (%s)' % (dt.strftime('%H:%M:%S'), before)

    def on_anchorClicked(self, url):
        self._updateDisplay()
        try:
            self.network_join_request.emit(int(url.toString()))
        except Exception:
            print(traceback.format_exc(1))
