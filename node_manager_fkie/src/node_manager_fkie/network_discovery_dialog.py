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
from datetime import datetime

import roslib
import roslib.message
import rospy
import threading
import socket

from master_discovery_fkie.udp import McastSocket
from master_discovery_fkie.master_discovery import Discoverer
import node_manager_fkie as nm

class NetworkDiscoveryDialog(QtGui.QDialog, threading.Thread):
  
  TIMEOUT = 0.1
  
  display_clear_signal = QtCore.Signal()
  display_append_signal = QtCore.Signal(str)
  status_text_signal = QtCore.Signal(str)
  network_join_request = QtCore.Signal(int)


  def __init__(self, default_mcast_group, default_port, networks_count, parent=None):
    '''
    Creates an input dialog.
    @param default_port: the default discovery port
    @type default_port: C{int}
    @param networks_count: the count of discovering ports
    @type networks_count: C{int}
    '''
    QtGui.QDialog.__init__(self, parent=parent)
    threading.Thread.__init__(self)
    self.setObjectName('NetworkDiscoveryDialog')
    self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
    self.setWindowFlags(QtCore.Qt.Window)
    self.setWindowTitle('Network Discovery')
    self.resize(728,512)
    self.verticalLayout = QtGui.QVBoxLayout(self)
    self.verticalLayout.setObjectName("verticalLayout")
    self.verticalLayout.setContentsMargins(1, 1, 1, 1)

    self.display = QtGui.QTextBrowser(self)
    self.display.setReadOnly(True)
    self.verticalLayout.addWidget(self.display);
    self.display_clear_signal.connect(self.display.clear)
    self.display_append_signal.connect(self.display.append)
    self.display.anchorClicked.connect(self.on_anchorClicked)

    self.status_label = QtGui.QLabel('0 messages', self)
    self.verticalLayout.addWidget(self.status_label)
    self.status_text_signal.connect(self.status_label.setText)
    
    self._networks_count = networks_count
    self.sockets = []
    for p in range(networks_count):
      msock = McastSocket(default_port+p, default_mcast_group)
      self.sockets.append(msock)
      msock.settimeout(self.TIMEOUT)
    self._running = True
    
    self._discovered = dict()
    
    self._hosts = dict() # resolution for hostname and address
    
    self.mutex = threading.RLock()
#    thread = threading.Thread(target=self._listen)
    self.setDaemon(True)
    self.start()
    

  def run(self):
    index = 0
    self.parent().masterlist_service.refresh(self.parent().getMasteruri(), False)
    while (not rospy.is_shutdown()) and self._running:
      msg = None
      address = None
      status_text = ''.join(['listen on network: ', str(index), ' (', str(self._networks_count), ')'])
      self.status_text_signal.emit(status_text)
      with self.mutex:
        while True:
          try:
            (msg, address) = self.sockets[index].recvfrom(1024)
            hostname = None
            force_update = False
            if not address is None:
              try:
                hostname = self._hosts[address[0]]
              except:
                hostname = nm.nameres().hostname(str(address[0]))
                if hostname is None or hostname == str(address[0]):
                  self.status_text_signal.emit(''.join(['resolve: ', str(address[0])]))
                  try:
                    (hostname, aliaslist, ipaddrlist) = socket.gethostbyaddr(str(address[0]))
                    nm.nameres().addInfo(None, hostname, hostname)
                    nm.nameres().addInfo(None, address[0], hostname)
                  except:
                    import traceback
                    print traceback.format_exc(1)
                    pass
                self._hosts[address[0]] = hostname
            if not msg is None:
              try:
                (version, msg_tuple) = Discoverer.msg2masterState(msg, address)
                if not self._discovered.has_key(index):
                  self._discovered[index] = dict()
                  force_update = True
                self._discovered[index][address] = (hostname, time.time())
              except Exception, e:
                import traceback
                print traceback.format_exc(1)
                pass
            if force_update:
              self._updateDisplay()
          except socket.timeout:
    #        rospy.logwarn("TIMOUT ignored")
            break
          except socket.error:
            import traceback
            rospy.logwarn("socket error: %s", traceback.format_exc(1))
            break
          except:
            break
      index += 1
      if index >= len(self.sockets):
        index = 0
        self._updateDisplay()
        self.parent().masterlist_service.refresh(self.parent().getMasteruri(), False)

  def closeEvent (self, event):
    self._running = False
    with self.mutex:
      for p in range(len(self.sockets)):
        try:
          self.sockets[p].close()
        except:
          pass
    QtGui.QDialog.closeEvent(self, event)

  def _updateDisplay(self):
    self.display_clear_signal.emit()
    text = '<div style="font-family:Fixedsys,Courier,monospace; padding:10px;">\n'
    for index, addr_dict in self._discovered.items():
      text = ''.join([text, 'Network <b>', str(index), '</b>: <a href="', str(index),'">join</a><dl>'])
      for addr, (hostname, ts) in addr_dict.items():
        text = ''.join([text, '<dt>', self._getTsStr(ts), '   <b><u>', str(hostname), '</u></b> ', str(addr), '</dt>\n'])
      text = ''.join([text, '</dl><br>'])
    text = ''.join([text, '</div>'])
    self.display_append_signal.emit(text)

  def _getTsStr(self, timestamp):
    dt = datetime.fromtimestamp(timestamp)
    diff = time.time()-timestamp
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
    return ''.join([dt.strftime('%H:%M:%S'), ' (', before, ')'])
  
  def on_anchorClicked(self, url):
    self._updateDisplay()
    try:
      self.network_join_request.emit(int(url.toString()))
    except:
      import traceback
      print traceback.format_exc(1)
