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

from typing import Dict
from typing import List
from typing import Text


class NodeDescription:
    def __init__(self, name: Text, *, is_executable: bool = False, composable_container: Text = '', composable_nodes: List[Text] = []):
        self.name = name
        self.is_executable = is_executable
        self.composable_container = composable_container
        # create a new array to a void to fill a default one
        self.composable_nodes = composable_nodes if composable_nodes else []


class Capability:

    def __init__(self, name: Text = '', *, namespace: Text = '', cap_type: Text = '', images: List[Text] = [], description: Text = '', nodes: List[Text] = []):
        '''
        Capabilities defined in launch file.

        :param str namespace: the ROS namespace of the capability.
        :param str name: the name of the capability.
        :param str type: the type of the capability.
        :param images: list of the images assigned to the this capability.
        :type images: [str]
        :param str description: the description of the capability.
        :param nodes: a list of nodes assigned to this group. The nodes are described by full ROS name (with namesspace).
        :type nodes: [str]
        '''
        self.namespace = namespace
        self.name = name
        self.type = cap_type
        # create a new array to a void to fill a default one
        self.images = images if images else []
        self.description = description
        # create a new array to a void to fill a default one
        self.nodes = nodes if nodes else []

    def __repr__(self):
        return "<%s[%s/%s], with %d nodes>" % (self.__class__, self.namespace, self.name, len(self.nodes))

    def __str__(self):
        if self.nodes:
            return "%s [%s]" % (self.__repr__(), ','.join([str(node) for node in self.nodes]))
        return self.__repr__()


class RobotDescription:

    def __init__(self, *, machine: Text = '', robot_name: Text = '', robot_type: Text = '', robot_images: List[Text] = [], robot_descr: Text = '', capabilities: List[Capability] = []):
        '''
        Description of the robot configured by this launch file.

         :param str machine: the address of the host.
         :param str robot_name: robot name.
         :param str robot_type: type of the robot.
         :param robot_images: list of the images assigned to the robot.
         :type robot_images: [str]
         :param str robot_descr: some description.
         :param capabilities: a list of capabilities :message:Capability.
         :type capabilities: [Capability]
        '''
        self.machine = machine
        self.robot_name = robot_name
        self.robot_type = robot_type
        # create a new array to a void to fill a default one
        self.robot_images = robot_images if robot_images else []
        self.robot_descr = robot_descr
        # create a new array to a void to fill a default one
        self.capabilities = capabilities if capabilities else []

    def __repr__(self):
        return "<%s[%s], machine=%s, with %d capabilities>" % (self.__class__, self.robot_name, self.machine, len(self.capabilities))

    def __str__(self):
        if self.capabilities:
            return "%s [%s]" % (self.__repr__(), ','.join([str(cap) for cap in self.capabilities]))
        return self.__repr__()


class LaunchDescription:

    def __init__(self, path='', nmduri: Text = '', *, nodes: List[NodeDescription] = [], robot_descriptions: List[RobotDescription] = []):
        '''
        Description of the robot configured by this launch file.

         :param str path: path of the launch file.
         :param str nmduri: if not empty, the nodes of this launch file are launched on specified host.
         :param nodes: list of node names.
         :type nodes: [NodeDescription]
         :param robot_descriptions: a list of capabilities :message:RobotDescription.
         :type robot_descriptions: [RobotDescription]
        '''
        self.path = path
        self.nmduri = nmduri
        # create a new array to a void to fill a default one
        self.nodes = nodes if nodes else []
        # create a new array to a void to fill a default one
        self.robot_descriptions = robot_descriptions if robot_descriptions else []

    def __repr__(self):
        return "<%s[%s, nmduri: %s], with %d nodes>" % (self.__class__, self.path, self.nmduri, len(self.nodes))

    def __str__(self):
        if self.nodes:
            return "%s [%s]" % (self.__repr__(), ','.join([str(node) for node in self.nodes]))
        return self.__repr__()
