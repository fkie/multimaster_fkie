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

import time
import threading

from python_qt_binding import QtCore
import rospy
import node_manager_fkie as nm
try:
#  from default_cfg_fkie.msg import Capability
  from multimaster_msgs_fkie.srv import ListDescription, ListNodes#, LoadLaunch, Task
except ImportError, e:
  import sys
  print >> sys.stderr, "Can't import services of default_cfg_fkie. Is default_cfg_fkie package compiled?"
  raise ImportError(str(e))

class DefaultConfigHandler(QtCore.QObject):
  '''
  A class to retrieve the list of nodes from the default configuration service. 
  The received node list will be published by sending a QT signal. To retrieve 
  the configuration a new thread will be created.
  '''
  node_list_signal = QtCore.Signal(str, str, list)
  '''
  node_list_signal is a signal, which is emitted, if a new list with nodes is 
  retrieved. The signal has the URI of the service, the name of the service and 
  a list with node names as parameter.
  '''
  description_signal = QtCore.Signal(str, str, list)
  '''
  description_signal is a signal, which is emitted, if a new list with descriptions is 
  retrieved. The signal has the URI of the service, the name of the service and 
  a list with descriptions (L{default_cfg_fkie.Description}) parameter.
  '''
  err_signal = QtCore.Signal(str, str, str)

  def __init__(self):
    QtCore.QObject.__init__(self)
    self.__serviceThreads = {}
    self._lock = threading.RLock()
  
  def stop(self):
    print "    Shutdown default config update threads..."
    for _, service in self.__serviceThreads.iteritems():
      service.join(3)
    print "    Default config update threads are off!"

  def requestNodeList(self, service_uri, service, delay_exec=0.0):
    '''
    This method starts a thread to get the informations about the default
    configured nodes. If all informations are retrieved, a C{node_list_signal} of 
    this class will be emitted. If for given service a thread is 
    already running, the request will be ignored.
    This method is thread safe. 
    
    @param service_uri: the URI of the service
    @type service_uri: C{str}
    @param service: the name of service to get the node list
    @type service: C{str}
    @param delay_exec: delayd the execution
    @type delay_exec: C{float}
    '''
    with self._lock:
      if not (self.__serviceThreads.has_key((service_uri, service))):
        upthread = ServiceThread(service_uri, service, delay_exec)
        upthread.update_signal.connect(self._on_node_list)
        upthread.err_signal.connect(self._on_err)
        self.__serviceThreads[(service_uri, service)] = upthread
        upthread.start()

  def requestDescriptionList(self, service_uri, service, delay_exec=0.0):
    '''
    This method starts a thread to get the descriptions from the default
    configuration node. If all informations are retrieved, a C{description_signal} of 
    this class will be emitted. If for given service a thread is 
    already running, the request will be ignored.
    This method is thread safe. 
    
    @param service_uri: the URI of the service
    @type service_uri: C{str}
    @param service: the name of service to get the description
    @type service: C{str}
    @param delay_exec: delayd the execution
    @type delay_exec: C{float}
    '''
    with self._lock:
      if not (self.__serviceThreads.has_key((service_uri, service))):
        upthread = ServiceDescriptionThread(service_uri, service, delay_exec)
        upthread.update_signal.connect(self._on_descr_list)
        upthread.err_signal.connect(self._on_err)
        self.__serviceThreads[(service_uri, service)] = upthread
        upthread.start()

  def _on_node_list(self, service_uri, service, nodes):
    with self._lock:
      try:
        thread = self.__serviceThreads.pop((service_uri, service))
        del thread
      except KeyError:
        pass
    self.node_list_signal.emit(service_uri, service, nodes)

  def _on_descr_list(self, service_uri, service, items):
    with self._lock:
      try:
        thread = self.__serviceThreads.pop((service_uri, service))
        del thread
      except KeyError:
        pass
    self.description_signal.emit(service_uri, service, items)

  def _on_err(self, service_uri, service, msg):
    with self._lock:
      try:
        thread = self.__serviceThreads.pop((service_uri, service))
        del thread
      except KeyError:
        pass
    self.err_signal.emit(service_uri, service, msg)



class ServiceThread(QtCore.QObject, threading.Thread):
  '''
  A thread to to retrieve the list of nodes from the default configuration 
  service and publish it by sending a QT signal.
  '''
  update_signal = QtCore.Signal(str, str, list)
  err_signal = QtCore.Signal(str, str, str)

  def __init__(self, service_uri, service, delay_exec=0.0, parent=None):
    QtCore.QObject.__init__(self)
    threading.Thread.__init__(self)
    self._service_uri = service_uri
    self._service = service
    self._delay_exec = delay_exec
    self.setDaemon(True)

  def run(self):
    '''
    '''
    if self._service and self._service_uri:
      try:
        if self._delay_exec > 0:
          time.sleep(self._delay_exec)
        _, resp = nm.starter().callService(self._service_uri, self._service, ListNodes)
        self.update_signal.emit(self._service_uri, self._service, resp.nodes)
      except:
        import traceback
        lines = traceback.format_exc(1).splitlines()
        rospy.logwarn("Error while retrieve the node list from %s[%s]: %s", str(self._service), str(self._service_uri), str(lines[-1]))
        self.err_signal.emit(self._service_uri, self._service, lines[-1])

class ServiceDescriptionThread(QtCore.QObject, threading.Thread):
  '''
  A thread to to retrieve the list with descriptions from the default configuration 
  service and publish it by sending a QT signal.
  '''
  update_signal = QtCore.Signal(str, str, list)
  err_signal = QtCore.Signal(str, str, str)

  def __init__(self, service_uri, service, delay_exec=0.0, parent=None):
    QtCore.QObject.__init__(self)
    threading.Thread.__init__(self)
    self._service_uri = service_uri
    self._service = service
    self._delay_exec = delay_exec
    self.setDaemon(True)

  def run(self):
    '''
    '''
    if self._service:
      try:
        if self._delay_exec > 0:
          time.sleep(self._delay_exec)
        _, resp = nm.starter().callService(self._service_uri, self._service, ListDescription)
        self.update_signal.emit(self._service_uri, self._service, [resp])
      except:
        import traceback
        lines = traceback.format_exc(1).splitlines()
        rospy.logwarn("Error while retrieve the description from %s[%s]: %s", str(self._service), str(self._service_uri), str(lines[-1]))
        self.err_signal.emit(self._service_uri, self._service, lines[-1])
