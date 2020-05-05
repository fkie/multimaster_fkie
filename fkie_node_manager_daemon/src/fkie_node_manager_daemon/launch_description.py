# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Fraunhofer FKIE/CMS, Alexander Tiderko
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




class LaunchDescription:

    def __init__(self, path='', masteruri='', host='', nodes=[], robot_descriptions=[], nodelets={}, associations={}):
        '''
        Description of the robot configured by this launch file.

         :param str path: path of the launch file.
         :param str masteruri: starts nodes of this file with specified ROS_MASTER_URI. If host is empty,
             the nodes are started on the host specified by hostname of the masteruri.
         :param str host: if not empty, the nodes of this launch file are launched on specified host.
         :param nodes: list of node names.
         :type nodes: [str]
         :param robot_descriptions: a list of capabilities :message:RobotDescription.
         :type robot_descriptions: [RobotDescription]
         :param nodelets: a dictionary with nodelets manager and controlled nodelet clients
         :type nodelets: {str: [str]}
         :param associations: a dictionary with associations of nodes
         :type associations: {str: [str]}
        '''
        self.path = path
        self.masteruri = masteruri
        self.host = host
        self.nodes = nodes if nodes else []  # create a new array to a void to fill a default one
        self.robot_descriptions = robot_descriptions if robot_descriptions else []  # create a new array to a void to fill a default one
        self.nodelets = nodelets if nodelets else {}
        self.associations = associations if associations else {}

    def __repr__(self):
        return "<%s[%s, masteruri: %s, host: %s], with %d nodes>" % (self.__class__, self.path, self.masteruri, self.host, len(self.nodes))

    def __str__(self):
        if self.nodes:
            return "%s [%s]" % (self.__repr__(), ','.join([str(node) for node in self.nodes]))
        return self.__repr__()


class RobotDescription:

    def __init__(self, machine='', robot_name='', robot_type='', robot_images=[], robot_descr='', capabilities=[]):
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
        self.robot_images = robot_images if robot_images else []  # create a new array to a void to fill a default one
        self.robot_descr = robot_descr
        self.capabilities = capabilities if capabilities else []  # create a new array to a void to fill a default one

    def __repr__(self):
        return "<%s[%s], machine=%s, with %d capabilities>" % (self.__class__, self.robot_name, self.machine, len(self.capabilities))

    def __str__(self):
        if self.capabilities:
            return "%s [%s]" % (self.__repr__(), ','.join([str(cap) for cap in self.capabilities]))
        return self.__repr__()


class Capability:

    def __init__(self, name='', namespace='', cap_type='', images=[], description='', nodes=[]):
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
        self.images = images if images else []  # create a new array to a void to fill a default one
        self.description = description
        self.nodes = nodes if nodes else []  # create a new array to a void to fill a default one

    def __repr__(self):
        return "<%s[%s/%s], with %d nodes>" % (self.__class__, self.namespace, self.name, len(self.nodes))

    def __str__(self):
        if self.nodes:
            return "%s [%s]" % (self.__repr__(), ','.join([str(node) for node in self.nodes]))
        return self.__repr__()
