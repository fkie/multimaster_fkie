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



from python_qt_binding.QtCore import QObject, Signal
import threading
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient


import rospy


class ParameterHandler(QObject):
    '''
    A class to retrieve the parameter list and their values from a ROS parameter
    server. The results are published by sending a QT signal. To parameter a new
    thread will be created.
    '''
    parameter_list_signal = Signal(str, int, str, list)
    '''
  parameter_list_signal is a signal, which is emitted if a new list with
  parameter names is retrieved. The signal has the URI of the ROS parameter
  server and (code, statusMessage, parameterNameList) - the response of the
  server U{http://www.ros.org/wiki/ROS/Parameter%20Server%20API#getParamNames}.
  '''
    parameter_values_signal = Signal(str, int, str, dict)
    '''
  parameter_values_signal is a signal, which is emitted if a new list with
  parameter names and their values is retrieved. The signal has the URI of the
  ROS parameter server and (code, statusMessage, parameterNameValueDictionary)
  as parameter. The dictionary has the format C{dict(paramName : (code, statusMessage, parameterValue))}.
  For details see U{http://www.ros.org/wiki/ROS/Parameter%20Server%20API#getParam}.
  '''

    delivery_result_signal = Signal(str, int, str, dict)
    '''
  delivery_result_signal a signal is emitted after the parameter value was
  set. For format see C{parameter_values_signal} signal. '''

    def __init__(self):
        QObject.__init__(self)
        self.setObjectName('ParameterHandler')
        self.__requestListThreads = []
        self.__requestValuesThreads = []
        self.__deliveryThreads = []
        self._lock = threading.RLock()
#    print '=============== create', self.objectName()
#
#  def __del__(self):
#    print "************ destroy", self.objectName()
#    print self.__requestListThreads
#    print self.__requestValuesThreads
#    print self.__deliveryThreads

    def requestParameterList(self, masteruri, ns='/'):
        '''
        This method starts a thread to get the parameter list from ROS parameter
        server. If all informations are retrieved, a C{parameter_list_signal} of
        this class will be emitted.
        This method is thread safe.

        @param masteruri: the URI of the ROS parameter server
        @type masteruri: C{str}
        @param ns: the namespace of delivered parameter (Default: /)
        @type ns: C{str}
        '''
        with self._lock:
            reqthread = RequestListThread(masteruri, ns)
            reqthread.parameter_list_signal.connect(self._on_param_list)
            self.__requestListThreads.append(reqthread)
            reqthread.start()

    def requestParameterValues(self, masteruri, params):
        '''
        This method starts a thread to get the parameter values from ROS parameter
        server. If all informations are retrieved, a C{parameter_values_signal} of
        this class will be emitted.
        This method is thread safe.

        @param masteruri: the URI of the ROS parameter server
        @type masteruri: C{str}
        @param params: List with parameter names
        @type params: C{[str]}
        '''
        with self._lock:
            reqthread = RequestValuesThread(masteruri, params)
            reqthread.parameter_values_signal.connect(self._on_param_values)
            self.__requestValuesThreads.append(reqthread)
            reqthread.start()

    def deliverParameter(self, masteruri, params):
        '''
        This method starts a thread to load the parameter values into ROS parameter
        server. If all informations are retrieved, a C{delivery_result_signal} of
        this class will be emitted.
        This method is thread safe.

        @param masteruri: the URI of the ROS parameter server
        @type masteruri: C{str}
        @param params: The dictinary the parameter name and their value, see U{http://www.ros.org/wiki/ROS/Parameter%20Server%20API#setParam}
        @type params: C{dict(str:value)}
        '''
        with self._lock:
            reqthread = DeliverValuesThread(masteruri, params)
            reqthread.result_signal.connect(self._on_set_result)
            self.__deliveryThreads.append(reqthread)
            reqthread.start()

    def _on_param_list(self, masteruri, code, msg, params):
        self.parameter_list_signal.emit(masteruri, code, msg, params)
        with self._lock:
            try:
                thread = self.__requestListThreads.pop(0)
                del thread
            except KeyError:
                pass

    def _on_param_values(self, masteruri, code, msg, values):
        self.parameter_values_signal.emit(masteruri, code, msg, values)
        with self._lock:
            try:
                thread = self.__requestValuesThreads.pop(0)
                del thread
            except KeyError:
                pass

    def _on_set_result(self, masteruri, code, msg, values):
        self.delivery_result_signal.emit(masteruri, code, msg, values)
        with self._lock:
            try:
                thread = self.__deliveryThreads.pop(0)
                del thread
            except KeyError:
                pass


class RequestListThread(QObject, threading.Thread):
    '''
    A thread to to retrieve the parameter list from ROSparameter server
    and publish it by sending a QT signal.
    '''
    parameter_list_signal = Signal(str, int, str, list)

    def __init__(self, masteruri, ns, parent=None):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._masteruri = masteruri
        self._ns = ns
        self.setDaemon(True)

    def run(self):
        '''
        '''
        if self._masteruri:
            try:
                name = rospy.get_name()
                master = xmlrpcclient.ServerProxy(self._masteruri)
                code, msg, params = master.getParamNames(name)
                # filter the parameter
                result = []
                for p in params:
                    if p.startswith(self._ns):
                        result.append(p)
                self.parameter_list_signal.emit(self._masteruri, code, msg, result)
            except Exception:
                import traceback
                err_msg = "Error while retrieve the parameter list from %s: %s" % (self._masteruri, traceback.format_exc(1))
                rospy.logwarn(err_msg)
#        lines = traceback.format_exc(1).splitlines()
                self.parameter_list_signal.emit(self._masteruri, -1, err_msg, [])


class RequestValuesThread(QObject, threading.Thread):
    '''
    A thread to to retrieve the value for given parameter from ROSparameter server
    and publish it by sending a QT signal.
    '''
    parameter_values_signal = Signal(str, int, str, dict)

    def __init__(self, masteruri, params, parent=None):
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._masteruri = masteruri
        self._params = params
        self.setDaemon(True)

    def run(self):
        '''
        '''
        if self._masteruri:
            result = dict()
            for p in self._params:
                result[p] = None
            try:
                name = rospy.get_name()
                master = xmlrpcclient.ServerProxy(self._masteruri)
                param_server_multi = xmlrpcclient.MultiCall(master)
                for p in self._params:
                    param_server_multi.getParam(name, p)
                r = param_server_multi()
                for index, (code, msg, value) in enumerate(r):
                    result[self._params[index]] = (code, msg, value)
                self.parameter_values_signal.emit(self._masteruri, 1, '', result)
            except Exception:
                import traceback
#        err_msg = "Error while retrieve parameter values from %s: %s"%(self._masteruri, traceback.format_exc(1))
#        rospy.logwarn(err_msg)
#        lines = traceback.format_exc(1).splitlines()
                self.parameter_values_signal.emit(self._masteruri, -1, traceback.format_exc(1), result)


class DeliverValuesThread(QObject, threading.Thread):
    '''
    A thread to to deliver the value for given parameter to ROSparameter server
    and publish the result by sending a QT signal.
    '''
    result_signal = Signal(str, int, str, dict)

    def __init__(self, masteruri, params, parent=None):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param params: The dictinary the parameter name and their value, see U{http://www.ros.org/wiki/ROS/Parameter%20Server%20API#setParam}
        @type params: C{dict(str: value)}
        '''
        QObject.__init__(self)
        threading.Thread.__init__(self)
        self._masteruri = masteruri
        self._params = params
        self.setDaemon(True)

    def run(self):
        '''
        '''
        if self._masteruri:
            result = dict()
            names = list(self._params.keys())
            for p in names:
                result[p] = None
            try:
                name = rospy.get_name()
                master = xmlrpcclient.ServerProxy(self._masteruri)
                param_server_multi = xmlrpcclient.MultiCall(master)
                for p, v in self._params.items():
                    param_server_multi.setParam(name, p, v)
                r = param_server_multi()
                for index, (code, msg, value) in enumerate(r):
                    result[names[index]] = (code, msg, value)
                self.result_signal.emit(self._masteruri, 1, '', result)
            except Exception:
                import traceback
                err_msg = "Error while deliver parameter values to %s: %s" % (self._masteruri, traceback.format_exc(1))
                rospy.logwarn(err_msg)
#        lines = traceback.format_exc(1).splitlines()
                self.result_signal.emit(self._masteruri, -1, err_msg, result)
