# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fkie_multimaster_msgs.msg import DiscoveredState
from .host import get_host_name
import fkie_node_manager_daemon as nmd


class RosSubscriberRosState():
    '''
    A class to receive ROS DiscoveredState messages and forward these as QtSignal.
    '''

    def create_subscription(self):
        '''
        This method creates a ROS subscriber to receive the ROS messages.
        The retrieved messages will be emitted as *_signal.
        '''
        self.sub_discovered_state = None
        self.topic_name = '%s/%s/rosstate' % (nmd.settings.NM_DISCOVERY_NAMESPACE, nmd.settings.NM_DISCOVERY_NAME)
        nmd.rosnode.get_logger().info('listen for discovered items on %s' % self.topic_name)
        qos_profile = QoSProfile(depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            # history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.sub_discovered_state = nmd.rosnode.create_subscription(DiscoveredState, self.topic_name, self._on_msg, qos_profile=qos_profile)

    def get_publisher_count(self):
        if hasattr(self, 'topic_name') and self.topic_name is not None:
            return nmd.rosnode.count_publishers(self.topic_name)
        return -1

    def stop(self):
        '''
        Unregister the subscribed topic
        '''
        if hasattr(self, 'sub_discovered_state') and self.sub_discovered_state is not None:
            nmd.rosnode.destroy_subscription(self.sub_discovered_state)
            del self.sub_discovered_state

    def _on_msg(self, msg:DiscoveredState):
        '''
        The method to handle the received Log messages.
        :param msg: the received message
        :type msg: fkie_multimaster_msgs.DiscoveredState<XXX>
        '''
        nmd.rosnode.get_logger().info('new message on %s' % self.topic_name)
