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
from rosgraph_msgs.msg import Log

import rospy


class RosoutListener(QObject):
    '''
    A class to receive the ROS master state updates from a ROS topic. The topic
    will be determine using U{fkie_master_discovery.interface_finder.get_changes_topic()<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#interface-finder-module>}.
    '''
    rosdebug_signal = Signal(Log)
    rosinfo_signal = Signal(Log)
    roswarn_signal = Signal(Log)
    roserr_signal = Signal(Log)
    rosfatal_signal = Signal(Log)

    def registerByROS(self):
        '''
        This method creates a ROS subscriber to received the notifications of ROS
        Logs. The retrieved messages will be emitted as *_signal.
        '''
        self.sub_rosout = None
        rospy.loginfo("listen for logs on %s", '/rosout_agg')
        self.sub_rosout = rospy.Subscriber('/rosout_agg', Log, self._on_log)

    def stop(self):
        '''
        Unregister the subscribed topic
        '''
        if hasattr(self, 'sub_rosout'):
            self.sub_rosout.unregister()
            del self.sub_rosout

    def _on_log(self, msg):
        '''
        The method to handle the received Log messages.
        @param msg: the received message
        @type msg: U{rosgraph_msgs.Log<http://docs.ros.org/kinetic/api/rosgraph_msgs/html/msg/Log.html>}
        '''
        if msg.name in [rospy.get_name(), '/node_manager_daemon', '/master_discovery', '/master_sync']:
            if msg.level == Log.DEBUG:
                self.rosdebug_signal.emit(msg)
            if msg.level == Log.INFO:
                self.rosinfo_signal.emit(msg)
            elif msg.level == Log.WARN:
                self.roswarn_signal.emit(msg)
            elif msg.level == Log.ERROR:
                self.roserr_signal.emit(msg)
            elif msg.level == Log.FATAL:
                self.rosfatal_signal.emit(msg)
