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

import socket
try:
    import cStringIO as io  # python 2 compatibility
except ImportError:
    import io
import roslib
import rospy

from .common import get_hostname
from .filter_interface import FilterInterface


class NodeInfo(object):
    '''
    The NodeInfo class stores informations about a ROS node.

    :param name: the name of the node

    :type name: str

    :param masteruri: the URI of the ROS master, where the node is registered.
                      This masteruri will be used to determine, whether the ROS master and the
                      node are running on the same machine.

    :type masteruri: str
    '''

    def __init__(self, name, masteruri):
        '''
        Creates a new NodeInfo for a node with given name.

        :param name: the name of the node

        :type name: str

        :param masteruri: the URI of the ROS master, where the node is registered.
                          This masteruri will be used to determine, whether the ROS master and the
                          node are running on the same machine.

        :type masteruri: str
        '''
        self.__name = name
        self.__masteruri = masteruri
        self.__org_masteruri = masteruri
        self.__uri = None
        self.pid = None
        '''the process id of the node. Invalid id has a ``None`` value'''
        self.__local = False
        self.__local_master = True
        self._publishedTopics = []
        self._subscribedTopics = []
        self._services = []

    def __repr__(self):
        return "<NodeInfo name=%s, uri=%s, masteruri=%s, is_local=%s, pub_topics=%d, sub_topics=%d>" % (self.name, self.uri, self.masteruri, self.isLocal, len(self.publishedTopics), len(self.subscribedTopics))

    @property
    def name(self):
        '''
        :return: the name of the node.

        :rtype: str
        '''
        return self.__name

    @property
    def uri(self):
        '''
        :return: the URI of the RPC API of the node.

        :rtype: str
        '''
        return self.__uri

    @uri.setter
    def uri(self, uri):
        '''
        Sets the URI of the RPC API of the node.
        '''
        self.__uri = uri
        self.__local = NodeInfo.local_(self.__masteruri, self.__org_masteruri, self.__uri)

    @property
    def masteruri(self):
        '''
        :return: the URI of the ROS master where the node is registered.

        :rtype: str
        '''
        return self.__org_masteruri

    @masteruri.setter
    def masteruri(self, uri):
        '''
        Sets the ROS master URI.
        '''
        self.__org_masteruri = uri
        self.__local = NodeInfo.local_(self.__masteruri, self.__org_masteruri, self.__uri)
        self.__local_master = (self.__masteruri == self.__org_masteruri)

    @property
    def isLocal(self):
        '''
        :return: ``True`` if the node and the ROS master are running on the same machine.

        :rtype: bool
        '''
        return self.__local

    @property
    def isLocalMaster(self):
        '''
        :return: ``True`` if the node is registered on the local machine.

        :rtype: bool
        '''
        return self.__local_master

    @property
    def publishedTopics(self):
        '''
        :return: the list of all published topics by this node.

        :rtype: list of strings
        '''
        return self._publishedTopics

    @publishedTopics.setter
    def publishedTopics(self, name):
        '''
        Append a new published topic to this node.

        :param name: the name of the topic

        :type name: str
        '''
        try:
            if isinstance(name, list):
                del self._publishedTopics
                self._publishedTopics = name
            else:
                self._publishedTopics.index(name)
        except ValueError:
            self._publishedTopics.append(name)

#  @publishedTopics.deleter
#  def publishedTopics(self):
#    del self._publishedTopics

    @property
    def subscribedTopics(self):
        '''
        :return: the list of all subscribed topics by this node.

        :rtype: list of strings
        '''
        return self._subscribedTopics

    @subscribedTopics.setter
    def subscribedTopics(self, name):
        '''
        Append a new subscribed topic to this node.

        :param name: the name of the topic

        :type name: str
        '''
        try:
            if isinstance(name, list):
                del self._subscribedTopics
                self._subscribedTopics = name
            else:
                self._subscribedTopics.index(name)
        except ValueError:
            self._subscribedTopics.append(name)

#  @subscribedTopics.deleter
#  def subscribedTopics(self):
#    del self._subscribedTopics

    @property
    def services(self):
        '''
        :return: the list of all services provided by this node.

        :rtype: list of strings
        '''
        return self._services

    @services.setter
    def services(self, name):
        '''
        Append a new service to this node.

        :param name: the name of the topic

        :type name: str
        '''
        try:
            if isinstance(name, list):
                del self._services
                self._services = name
            else:
                self._services.index(name)
        except ValueError:
            self._services.append(name)

#  @services.deleter
#  def services(self):
#    del self._services

    def copy(self, new_masteruri=None):
        '''
        Creates a copy of this object and returns it.

        :param new_masteruri: the masteruri of the new masterinfo
        :rtype: :mod:`fkie_master_discovery.master_info.NodeInfo`
        '''
        if new_masteruri is None:
            new_masteruri = self.masteruri
        result = NodeInfo(self.name, new_masteruri)
        result.uri = str(self.uri) if self.uri is not None else None
        result.masteruri = self.masteruri
        result.pid = self.pid
        result._publishedTopics = list(self._publishedTopics)
        result._subscribedTopics = list(self._subscribedTopics)
        result._services = list(self._services)
        return result

    @staticmethod
    def local_(masteruri, org_masteruri, uri):
        '''
        Test the node whether it's run on the same machineas the ROS master and ``masteruri`` and ``org_masteruri`` are equal.

        :param masteruri: The URI of the ROS master currently tested.

        :type masteruri: str

        :param org_masteruri: The URI of the ROS master, where the node was originally registered.

        :type org_masteruri: str

        :param uri: The URI of the node.

        :type uri: str

        :rtype: bool
        '''
        result = False
        try:
            om = get_hostname(masteruri)
            on = get_hostname(uri)
            result = (om == on) and (masteruri == org_masteruri)
        except:
            pass
        return result


class TopicInfo(object):
    '''
    The TopicInfo class stores informations about a ROS topic.

    :param name: the name of the topic

    :type name: str
    '''

    def __init__(self, name):
        '''
        Creates a new TopicInfo for a topic with given name.

        :param name: the name of the topic

        :type name: str
        '''
        self.__name = name
        self.type = None
        '''the type of the topic. (Default: ``None``)'''
        self._publisherNodes = []
        self._subscriberNodes = []

    @property
    def name(self):
        '''
        :return: the name of the topic.

        :rtype: str
        '''
        return self.__name

    @property
    def publisherNodes(self):
        '''
        :return: the list with node names witch are publishing to this topic.

        :rtype: list of strings
        '''
        return list(self._publisherNodes)

    @publisherNodes.setter
    def publisherNodes(self, name):
        '''
        Append a new publishing node to this topic.
        '''
        try:
            if isinstance(name, list):
                del self._publisherNodes
                self._publisherNodes = name
            else:
                self._publisherNodes.index(name)
        except ValueError:
            self._publisherNodes.append(name)

#  @publisherNodes.deleter
#  def publisherNodes(self):
#    del self._publisherNodes

    @property
    def subscriberNodes(self):
        '''
        :return: the list with node names witch are subscribed to this topic.

        :rtype: list of strings
        '''
        return list(self._subscriberNodes)

    @subscriberNodes.setter
    def subscriberNodes(self, name):
        '''
        Append a new subscribing node to this topic.
        '''
        try:
            if isinstance(name, list):
                del self._subscriberNodes
                self._subscriberNodes = name
            else:
                self._subscriberNodes.index(name)
        except ValueError:
            self._subscriberNodes.append(name)

#  @subscriberNodes.deleter
#  def subscriberNodes(self):
#    del self._subscriberNodes

    def copy(self):
        '''
        Creates a copy this object and returns it.

        :rtype: :mod:`fkie_master_discovery.master_info.TopicInfo`
        '''
        result = TopicInfo(self.name)
        result.type = self.type
        result._publisherNodes = list(self._publisherNodes)
        result._subscriberNodes = list(self._subscriberNodes)
        return result


class ServiceInfo(object):
    '''
    The ServiceInfo class stores informations about a ROS service.

    :param name: the name of the service

    :type name: str

    :param masteruri: the URI of the ROS master, where the service is registered.
                      This masteruri will be used to determine, whether the ROS master and the
                      service are running on the same machine.

    :type masteruri: str
    '''

    def __init__(self, name, masteruri):
        '''
        Creates a new instance of the ServiceInfo.

        :param name: the name of the service

        :type name: str

        :param masteruri: the URI of the ROS master, where the service is registered.
                          This masteruri will be used to determine, whether the ROS master and the
                          service are running on the same machine.

        :type masteruri: str
        '''
        self.__name = name
        self.__masteruri = masteruri
        self.__org_masteruri = masteruri
        self.__uri = None
        self.__local = False
        self.__local_master = True
        self.type = None
        '''the type of the service. (Default: ``None``)'''
        self.__service_class = None
        self.args = None
        self._serviceProvider = []

    @property
    def name(self):
        '''
        :return: the name of the service.

        :rtype: str
        '''
        return self.__name

    @property
    def uri(self):
        '''
        :return: the URI of the RPC API of the service

        :rtype: str
        '''
        return self.__uri

    @uri.setter
    def uri(self, uri):
        '''
        Sets the uri of the service RPC interface and determine whether this service
        and the ROS master are running on the same machine.

        :param uri: The URI of the service RPC interface

        :type uri: str
        '''
        self.__uri = uri
        self.__local = NodeInfo.local_(self.__masteruri, self.__org_masteruri, self.__uri)

    @property
    def masteruri(self):
        '''
        :return: the URI of the ROS master of the service

        :rtype: str
        '''
        return self.__org_masteruri

    @masteruri.setter
    def masteruri(self, uri):
        '''
        Sets the uri of the origin ROS master and determine whether this service
        and the ROS master are running on the same machine.

        :param uri: The URI of the ROS master

        :type uri: str
        '''
        self.__org_masteruri = uri
        self.__local_master = (self.__masteruri == self.__org_masteruri)
        self.__local = NodeInfo.local_(self.__masteruri, self.__org_masteruri, self.__uri)

    @property
    def isLocal(self):
        '''
        :return: `True`, if this service and the master are on the same machine.
                 This will be determine on setting the uri-parameter.

        :rtype: bool
        '''
        return self.__local

    @property
    def isLocalMaster(self):
        '''
        :return: ``True`` if the service is registered on the local machine.

        :rtype: bool
        '''
        return self.__local_master

    @property
    def serviceProvider(self):
        '''
        :return: the list of the node names, which provide this service.

        :rtype: list of strings
        '''
        return self._serviceProvider

    @serviceProvider.setter
    def serviceProvider(self, name):
        '''
        Adds a new service provider, if no one with given name exists.

        :param name: name of the new service provider

        :type name: str
        '''
        try:
            self._serviceProvider.index(name)
        except ValueError:
            self._serviceProvider.append(name)

    @serviceProvider.deleter
    def serviceProvider(self):
        del self._serviceProvider

    def get_service_class(self, allow_get_type=False):
        '''
        Get the service class using the type of the service. NOTE: this
        method is from `rosservice` and changed to avoid a probe call to the service.

        :param allow_get_type: allow to connect to service and get the type if the type is not valid (in case of other host)

        :type allow_get_type: bool

        :return: service class

        :rtype: ServiceDefinition: service class

        :raise: ``ROSServiceException``, if service class cannot be retrieved
        '''
        if self.__service_class is not None:
            return self.__service_class

        srv_type = self.type
        # request the type if it is empty and allowed
        if not srv_type and allow_get_type and self.uri:
            dest_addr = dest_port = None
            try:
                dest_addr, dest_port = rospy.parse_rosrpc_uri(self.uri)
            except:
                pass
            else:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    # connect to service and probe it to get the headers
                    s.settimeout(0.5)
                    s.connect((dest_addr, dest_port))
                    header = {'probe': '1', 'md5sum': '*',
                              'callerid': rospy.get_name(), 'service': self.name}
                    roslib.network.write_ros_handshake_header(s, header)
                    srv_type = roslib.network.read_ros_handshake_header(s, io.StringIO(), 2048)
                    srv_type = srv_type['type']
                except socket.error:
                    pass
                except:
                    pass
                finally:
                    if s is not None:
                        s.close()

        import rosservice
        if not srv_type:
            raise rosservice.ROSServiceException("Not valid type of service [%s]." % str(srv_type))

        # get the Service class so we can populate the request
        service_class = roslib.message.get_service_class(srv_type)

        # #1083: roscpp services are currently returning the wrong type
        if service_class and self.type.endswith('Request') and \
                not hasattr(service_class, "_request_class"):
            srv_type = srv_type[:-7]
            service_class = roslib.message.get_service_class(srv_type)

        if service_class is None:
            pkg = roslib.names.resource_name_package(self.type)
            raise rosservice.ROSServiceException("Unable to load type [%s].\n" % self.type +
                                                 "Have you typed 'make' in [%s]?" % pkg)
        self.__service_class = service_class
        return service_class

    def copy(self, new_masteruri=None):
        '''
        Creates a copy of this object and returns it.

        :param new_masteruri: the masteruri of the new masterinfo
        :rtype: :mod:`fkie_master_discovery.master_info.NodeInfo`
        '''
        if new_masteruri is None:
            new_masteruri = self.masteruri
        result = ServiceInfo(self.name, new_masteruri)
        result.uri = self.uri
        result.masteruri = self.masteruri
        result.type = self.type
        result.args = self.args
        result._serviceProvider = list(self.serviceProvider)
        return result


class MasterInfo(object):
    '''
    The MasterInfo class stores informations about a ROS master.
    Not thread safe!

    :param masteruri: The URI of the corresponding master

    :type masteruri: str

    :param mastername: The name of the ROS master. If no one is given, it will be
                       extracted from the masteruri.

    :type mastername: str or ``None`` (Default: ``None``)
    '''

    def __init__(self, masteruri, mastername=None):
        '''
        Creates a new instance of the MasterInfo. The mastername will be extracted
        from the masterui, if no name is given.

        :param masteruri: The URI of the corresponding master

        :type masteruri: str

        :param mastername: The name of the ROS master. If no one is given, it will be
                           extracted from the masteruri.

        :type mastername: str or ``None`` (Default: ``None``)
        '''
        self.__masteruri = masteruri
        self.__mastername = mastername
        if mastername is None:
            self.__mastername = get_hostname(self.__masteruri)
        self.__nodelist = {}
        self.__topiclist = {}
        self.__servicelist = {}
        self.__timestamp = 0
        self.__timestamp_local = 0
        self.check_ts = 0
        '''the last time, when the state of the ROS master retrieved'''

    @staticmethod
    def from_list(l):
        '''
        Creates a new instance of the MasterInfo from given list.

        :param l: the list returned by :mod:`fkie_master_discovery.master_info.MasterInfo.listedState()`

        :type l: list

        :return: the new instance of the MasterInfo filled from list.

        :rtype: :mod:`fkie_master_discovery.master_info.MasterInfo`
        '''
        if l is None:
            return None
        result = MasterInfo(l[2], l[3])
        result.timestamp = float(l[0])
        result.timestamp_local = float(l[1])
        publishers = l[4]
        subscribers = l[5]
        services = l[6]
        topicTypes = l[7]
        nodes = l[8]
        serviceProvider = l[9]
        # set the publishers
        for pub, nodelist in publishers:
            result.topics = pub
            for n in nodelist:
                result.nodes = n
                result.getNode(n).publishedTopics = pub
                result.getTopic(pub).publisherNodes = n
        # set the subscribers
        for sub, nodelist in subscribers:
            result.topics = sub
            for n in nodelist:
                result.nodes = n
                result.getNode(n).subscribedTopics = sub
                result.getTopic(sub).subscriberNodes = n
        # set the services
        for s, provider in services:
            result.services = s
            for n in provider:
                result.nodes = n
                result.getNode(n).services = s
                result.getService(s).serviceProvider = n
        # set the topic types
        for topic, ttype in topicTypes:
            result.topics = topic
            result.getTopic(topic).type = ttype
        # set the node informations
        for nodename, uri, masteruri, pid, _local in nodes:
            result.nodes = nodename
            result.getNode(nodename).uri = uri
            result.getNode(nodename).masteruri = masteruri
            result.getNode(nodename).pid = pid
        # set the service informations
        for servicename, uri, masteruri, stype, _local in serviceProvider:
            result.services = servicename
            result.getService(servicename).uri = uri
            result.getService(servicename).masteruri = masteruri
            result.getService(servicename).type = stype
        return result

    @property
    def mastername(self):
        '''
        :return: the name of the ROS master. In most cases the ROS master name is the
                 name of the host, where the ROS master running. Although it can differ.

        :rtype: str
        '''
        return self.__mastername

    @property
    def masteruri(self):
        '''
        :return: the URI of the ROS master.

        :rtype: str
        '''
        return self.__masteruri

    @property
    def timestamp(self):
        '''
        :return: The timestamp when this MasterInfo was first time filled with the
                 information. See :mod:`fkie_master_discovery.master_info.MasterInfo.check_ts()`
                 to get the time, when the information was compared with the data of ROS Master.

        :rtype: float
        '''
        return self.__timestamp

    @timestamp.setter
    def timestamp(self, ts):
        '''
        Sets the timestamp of this instance

        :param ts: the new timestamp

        :type ts: float
        '''
        self.__timestamp = ts
        self.check_ts = ts
        self.__timestamp_local = ts

    @property
    def timestamp_local(self):
        '''
        :return: The timestamp when this MasterInfo was first time filled with the
                 information. See :mod:`fkie_master_discovery.master_info.MasterInfo.check_ts()`
                 to get the time, when the information was compared with the data of ROS Master.
                 This timestamp is only updated, not synchronized nodes, topics or services are
                 changed.

        :rtype: float
        '''
        return self.__timestamp_local

    @timestamp_local.setter
    def timestamp_local(self, ts):
        '''
        Sets the timestamp of this instance

        :param ts: the new timestamp

        :type ts: float
        '''
        self.__timestamp_local = ts

    @property
    def nodes(self):
        '''
        :return: the dictionary with ``node names`` and corresponding instances of ``NodeInfo``.

        :rtype: dict of (str : :mod:`fkie_master_discovery.master_info.NodeInfo`)
        '''
        return self.__nodelist

    @nodes.setter
    def nodes(self, name):
        '''
        Adds a new :mod:`fkie_master_discovery.master_info.NodeInfo` with given name.

        :note: If the NodeInfo already exists, do nothing.

        :param name: the name of new :mod:`fkie_master_discovery.master_info.NodeInfo`

        :type name: str
        '''
        if (name is None) or not name:
            return None
        if not (name in self.__nodelist):
            self.__nodelist[name] = NodeInfo(name, self.__masteruri)

    @property
    def node_names(self):
        '''
        :return: the list with node names

        :rtype: list of strings
        '''
#    @return: the list with node names
        return list(self.__nodelist.keys())

    @property
    def node_uris(self):
        '''
        :return: the list with node URI's.

        :rtype: list of strings
        '''
        uris = []
        for node in self.__nodelist.values():
            uris.append(node.uri)
        return uris

    @property
    def topics(self):
        '''
        :return: the dictionary with ``topic names`` and corresponding ``TopicInfo`` instances.

        :rtype: dict of (str : :mod:`fkie_master_discovery.master_info.TopicInfo`)
        '''
        return self.__topiclist

    @topics.setter
    def topics(self, name):
        '''
        Adds a new TopicInfo with given name. If the ``TopicInfo`` already exists, do
        nothing.

        :param name: the name of new :mod:`fkie_master_discovery.master_info.TopicInfo`

        :type name: str
        '''
        if (name is None) or not name:
            return None
        if not (name in self.__topiclist):
            self.__topiclist[name] = TopicInfo(name)

    @property
    def topic_names(self):
        '''
        :return: the list with topic names.

        :rtype: list of strings
        '''
        return list(self.__topiclist.keys())

    @property
    def services(self):
        '''
        :return: the dictionary with ``service names`` and corresponding ``ServiceInfo`` instances.

        :rtype: dict of (str : :mod:`fkie_master_discovery.master_info.ServiceInfo`)
        '''
        return self.__servicelist

    @services.setter
    def services(self, name):
        '''
        Adds a new :mod:`fkie_master_discovery.master_info.ServiceInfo` with given name. If the ServiceInfo already exists, do
        nothing.

        :param name: the name of new :mod:`fkie_master_discovery.master_info.ServiceInfo`

        :type name: str
        '''
        if (name is None) or not name:
            return None
        if not (name in self.__servicelist):
            self.__servicelist[name] = ServiceInfo(name, self.__masteruri)

    @property
    def service_names(self):
        '''
        :return: the list with service names.

        :rtype: list of strings
        '''
        return list(self.__servicelist.keys())

    @property
    def service_uris(self):
        '''
        :return: the list with service URI's.

        :rtype: list of strings
        '''
        uris = []
        for service in self.__servicelist.values():
            uris.append(service.uri)
        return uris

    def getNode(self, name):
        '''
        :param name: the name of the node

        :type name: str

        :return: the instance of the :mod:`fkie_master_discovery.master_info.NodeInfo` with given name

        :rtype: :mod:`fkie_master_discovery.master_info.NodeInfo` or ``None``
        '''
        if (name is None) or not name:
            return None
        return self.__nodelist.get(name, None)

    def getNodeEndsWith(self, suffix):
        '''
        Returns the node, which name ends with given suffix. On more then one node, only the fist found will be returned.

        :param suffix: the end of the name

        :type suffix: str

        :return: the instance of the :mod:`fkie_master_discovery.master_info.NodeInfo` with with given suffix

        :rtype: :mod:`fkie_master_discovery.master_info.NodeInfo` or ``None``
        '''
        if (suffix is None) or not suffix:
            return None
        for name, node in self.__nodelist.items():
            if name.endswith(suffix):
                return node
        return None

    def getTopic(self, name):
        '''
        Returns the topics with given name.

        :param name: the name of the topic

        :type name: str

        :return: the instance of the :mod:`fkie_master_discovery.master_info.TopicInfo` with given name.

        :rtype: :mod:`fkie_master_discovery.master_info.TopicInfo` or ``None``
        '''
        if (name is None) or not name:
            return None
        return self.__topiclist.get(name, None)

    def getService(self, name):
        '''
        Returns the service with given name.

        :param name: the name of the service

        :type name: str

        :return: the instance of the :mod:`fkie_master_discovery.master_info.ServiceInfo` with given name

        :rtype: :mod:`fkie_master_discovery.master_info.ServiceInfo` or ``None``
        '''
        if (name is None) or not name:
            return None
        return self.__servicelist.get(name, None)

    def __eq__(self, other):
        '''
        Compares the master state with other master state. The timestamp will not be
        compared.

        :param other: the another MasterInfo instance.

        :type other: :mod:`fkie_master_discovery.master_info.MasterInfo`

        :return: ``True``, if the states are equal.

        :rtype: boolean
        '''
#    import os                                ###################
#    cputimes = os.times()                    ###################
#    cputime_init = cputimes[0] + cputimes[1] ###################
#    try:
        if (other is None):
            return False
        if (self.masteruri != other.masteruri):
            return False
        if (set(self.node_uris) ^ set(other.node_uris)):
            return False
#    if (set(self.node_names) ^ set(other.node_names)):
#      return False
#    if (set(self.service_names) ^ set(other.service_names)):
#      return False
        if (set(self.service_uris) ^ set(other.service_uris)):
            return False
        if (set(self.topic_names) ^ set(other.topic_names)):
            return False
        # test for changes of each node parameter
        for name in self.node_names:
            n1 = self.getNode(name)
            n2 = other.getNode(name)
            if n1 is not None and n2 is not None:
                if n1.pid != n2.pid:
                    return False
#        if n1.uri != n2.uri:
#          return False
                if set(n1.publishedTopics) ^ set(n2.publishedTopics):
                    return False
                if set(n1.subscribedTopics) ^ set(n2.subscribedTopics):
                    return False
                if set(n1.services) ^ set(n2.services):
                    return False
        return True
#    finally:
#      cputimes = os.times() ###################
#      print "EQ:", (cputimes[0] + cputimes[1] - cputime_init), ", count nodes:", len(self.node_names) ###################

    def __ne__(self, other):
        return not self.__eq__(other)

    def has_local_changes(self, other):
        '''
        Compares the master state with other master state. The timestamp will not be
        compared.

        :param other: the another ``MasterInfo`` instance.

        :type other: :mod:`fkie_master_discovery.master_info.MasterInfo`

        :return: a tupel with two boolean values (all equal, only local equal)

        :rtype: (bool, bool)
        '''
#    import os                                ###################
#    cputimes = os.times()                    ###################
#    cputime_init = cputimes[0] + cputimes[1] ###################
#    try:
        if (other is None):
            return True
        if (self.masteruri != other.masteruri):
            return True
        # test for nodes
        node_names = list((set(self.node_names) | set(other.node_names)))
        for name in node_names:
            n1 = self.getNode(name)
            n2 = other.getNode(name)
            if n1 is not None and n2 is not None:
                if n1.isLocal or n1.isLocalMaster:
                    if n1.pid != n2.pid:
                        return True
                    if n1.uri != n2.uri:
                        return True
                    if set(n1.publishedTopics) ^ set(n2.publishedTopics):
                        return True
                    if set(n1.subscribedTopics) ^ set(n2.subscribedTopics):
                        return True
                    if set(n1.services) ^ set(n2.services):
                        return True
                # after local start of asynchronized node. The next check of master state return the local node.
                elif n2.isLocal or n2.isLocalMaster:
                    return True
            elif n1 is not None:
                if n1.isLocal or n1.isLocalMaster:
                    return True
            elif n2 is not None:
                if n2.isLocal or n2.isLocalMaster:
                    return True

        # test for services
        service_names = list(set(self.service_uris) | set(other.service_uris))
        for name in service_names:
            s1 = self.getService(name)
            s2 = other.getService(name)
            if s1 is not None and s2 is not None:
                if s1.isLocal or s1.isLocalMaster:
                    if s1.uri != s2.uri:
                        return True
                elif s2.isLocal or s2.isLocalMaster:
                    return True
            elif s1 is not None:
                if s1.isLocal or s1.isLocalMaster:
                    return True
            elif s2 is not None:
                if s2.isLocal or s2.isLocalMaster:
                    return True
        return False
#    finally:
#      cputimes = os.times() ###################
#      print "CHANGES:", (cputimes[0] + cputimes[1] - cputime_init), ", count nodes:", len(self.node_names) ###################

    def listedState(self, filter_interface=None):
        '''
        Returns a extended ROS Master State.

        :param filter_interface: The filter used to filter the nodes, topics or serivces out.

        :type filter_interface: FilterInterface

        :return: complete ROS Master State as

                 ``(stamp, stamp_local, masteruri, name, publishers, subscribers, services, topicTypes, nodes, serviceProvider)``

                   - ``publishers`` is of the form

                     ``[ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]``

                   - ``subscribers`` is of the form

                     ``[ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]``

                   - ``services`` is of the form

                     ``[ [service1, [service1Provider1...service1ProviderN]] ... ]``

                   - ``topicTypes`` is a list of

                     ``[ [topicName1, topicType1], ... ]``

                   - ``nodes`` is a list of (the pid of remote Nodes will not be resolved)

                     ``[nodename, XML-RPC URI, origin ROS_MASTER_URI, pid, {local, remote}]``

                   - ``serviceProvider`` is a list of (the type, serviceClass and args of remote Services will not be resolved)

                     ``[service, XML-RPC URI, origin ROS_MASTER_URI, type, {local, remote}]``


        :rtype: (``float``,
                 ``float``,
                 ``str``,
                 ``str``,
                 ``[ [str,[str] ] ]``,
                 ``[ [str,[str] ] ]``,
                 ``[ [str,[str] ] ]``,
                 ``[ [str,str] ]``,
                 ``[ [str,str,str,int,str] ]``,
                 ``[ [str,str,str,str,str] ])``
        '''
        iffilter = filter_interface
        if iffilter is None:
            iffilter = FilterInterface.from_list()
        stamp = '%.9f' % self.timestamp
        stamp_local = '%.9f' % self.timestamp_local
        publishers = []
        subscribers = []
        services = []
        topicTypes = []
        nodes = []
        serviceProvider = []
        added_nodes = []
        nodes_last_check = set()

        # filter the topics
        for name, topic in self.topics.items():
            pn = []
            for n in topic.publisherNodes:
                if not iffilter.is_ignored_publisher(n, name, topic.type):
                    pn.append(n)
                    nodes_last_check.add(n)
            if pn:
                publishers.append((name, pn))
            sn = []
            for n in topic.subscriberNodes:
                if not iffilter.is_ignored_subscriber(n, name, topic.type):
                    sn.append(n)
                    nodes_last_check.add(n)
            if sn:
                subscribers.append((name, sn))
            if pn or sn:
                topicTypes.append((name, topic.type))

        # filter the services
        for name, service in self.services.items():
            srv_prov = []
            for sp in service.serviceProvider:
                if not iffilter.is_ignored_service(sp, name):
                    srv_prov.append(sp)
                    nodes_last_check.add(sp)
            if srv_prov:
                services.append((name, srv_prov))
                serviceProvider.append((name, service.uri, str(service.masteruri), service.type if service.type is not None else '', 'local' if service.isLocal else 'remote'))

        # creates the nodes list
        for name, node in self.nodes.items():
            if name in nodes_last_check:
                nodes.append((name, node.uri, str(node.masteruri), node.pid, 'local' if node.isLocal else 'remote'))

        return (stamp, stamp_local, self.masteruri, self.mastername, publishers, subscribers, services, topicTypes, nodes, serviceProvider)

#  def __str__(self):
#    return str(self.listedState())

    def updateInfo(self, other):
        '''
        Updates the information about nodes, topics and services. If the other
        masterinfo is from the same ROS Master all informations are copied. If other
        contains the info from remote ROS Master, only the informations for
        synchronized nodes, topics or services are copied.
        :param other: the new master information object
        :type other: MasterInfo
        :return: The tuple of sets with added, changed and removed nodes, topics and services
        :rtype: (nodes_added, nodes_changed, nodes_removed, topics_added, topics_changed, topics_removed, services_added, services_changed, services_removed)
        '''
        if other is None:
            return

        topics_added = set()
        topics_changed = set()
        topics_removed = set()

        local_info = (self.masteruri == other.masteruri)
        if local_info:
            self.timestamp = other.timestamp
            self.timestamp_local = other.timestamp_local
            self.check_ts = other.check_ts
            # update topics
            own_topics_set = set(self.__topiclist.keys())
            other_topics_set = set(other.__topiclist.keys())
            topics_removed = own_topics_set - other_topics_set
            for t in topics_removed:
                del self.__topiclist[t]
            common_topics = own_topics_set & other_topics_set
            for t in common_topics:
                own_topic = self.__topiclist[t]
                other_topic = other.__topiclist[t]
                if own_topic.type != other_topic.type:
                    topics_changed.add(t)
                    own_topic.type = other_topic.type
                if set(own_topic._publisherNodes) ^ set(other_topic.publisherNodes):
                    topics_changed.add(t)
                    own_topic._publisherNodes = other_topic.publisherNodes
                if set(own_topic._subscriberNodes) ^ set(other_topic.subscriberNodes):
                    topics_changed.add(t)
                    own_topic._subscriberNodes = other_topic.subscriberNodes
            topics_added = other_topics_set - own_topics_set
            for t in topics_added:
                self.__topiclist[t] = other.__topiclist[t]

        nodes_changed = set()
        services_changed = set()

        # UPDATE NODES
        # get local nodes from other
        own_remote_nodes = dict()
        other_local_nodes = dict()
        for nodename, n in self.nodes.items():
            if n.masteruri == other.masteruri or self.masteruri == other.masteruri:
                own_remote_nodes[nodename] = n
        for nodename, n in other.nodes.items():
            if n.isLocal or self.masteruri == other.masteruri:
                other_local_nodes[nodename] = n
        # remove nodes
        own_remote_nodes_set = set(own_remote_nodes.keys())
        other_local_nodes_set = set(other_local_nodes.keys())
        nodes2remove = own_remote_nodes_set - other_local_nodes_set
        if nodes2remove:
            if local_info:
                for n in nodes2remove:
                    del self.__nodelist[n]
            else:
                pass
                # if the node is in own master_info, but not in remote, replace only the masteruri.
                # perhaps, if will be removed soon by master_sync
#        for n in nodes2remove:
#          own_remote_nodes[n].masteruri = self.masteruri
        # update nodes
        nodes2update = own_remote_nodes_set & other_local_nodes_set
        if nodes2update:
            for n in nodes2update:
                own_node = own_remote_nodes[n]
                other_node = other_local_nodes[n]
                if other_node.isLocal:
                    if own_node.pid != other_node.pid:
                        nodes_changed.add(n)
                        own_node.pid = other_node.pid
                    if own_node.uri != other_node.uri:
                        nodes_changed.add(n)
                        own_node.uri = other_node.uri
                    # update subscriptions of nodes
                    if set(own_node._publishedTopics) ^ set(other_node.publishedTopics):
                        nodes_changed.add(n)
                        own_node._publishedTopics = other_node.publishedTopics
                    if set(own_node._subscribedTopics) ^ set(other_node.subscribedTopics):
                        nodes_changed.add(n)
                        own_node._subscribedTopics = other_node.subscribedTopics
                    if set(own_node._services) ^ set(other_node.services):
                        nodes_changed.add(n)
                        own_node._services = other_node.services
        # add new nodes
        nodes_added = set()
        if local_info:
            nodes2add = other_local_nodes_set - own_remote_nodes_set
            if nodes2add:
                for n in nodes2add:
                    if not (n in self.__nodelist):
                        nodes_added.add(n)
                        self.__nodelist[n] = other_local_nodes[n].copy(self.masteruri)

        # UPDATE SERVICES
        own_remote_srvs = dict()
        other_local_srvs = dict()
        for srvname, s in self.services.items():
            if s.masteruri == other.masteruri or self.masteruri == other.masteruri:
                own_remote_srvs[srvname] = s
        for srvname, s in other.services.items():
            if s.isLocal or self.masteruri == other.masteruri:
                other_local_srvs[srvname] = s
        own_remote_srvs_set = set(own_remote_srvs.keys())
        other_local_srvs_set = set(other_local_srvs.keys())
        # remove services
        srvs2remove = own_remote_srvs_set - other_local_srvs_set
        if srvs2remove:
            if local_info:
                for s in srvs2remove:
                    del self.__servicelist[s]
            else:
                # if the service is in own master_info, but not in remote, replace only the masteruri.
                # perhaps, if will be removed soon by master_sync
                for s in srvs2remove:
                    own_remote_srvs[s].masteruri = self.masteruri
        # update services
        srv2update = own_remote_srvs_set & other_local_srvs_set
        if srv2update:
            for s in srv2update:
                own_srv = own_remote_srvs[s]
                other_srv = other_local_srvs[s]
                if other_srv.isLocal:
                    if own_srv.type != other_srv.type:
                        services_changed.add(s)
                        own_srv.type = other_srv.type
                    if own_srv.uri != other_srv.uri:
                        services_changed.add(s)
                        own_srv.uri = other_srv.uri
                    if own_srv.args != other_srv.args:
                        services_changed.add(s)
                        own_srv.args = other_srv.args
                    # update provider
                    if set(own_srv._serviceProvider) ^ set(other_srv.serviceProvider):
                        services_changed.add(s)
                        own_srv._serviceProvider = other_srv.serviceProvider
        # add new services
        srvs_added = set()
        if local_info:
            srv2add = other_local_srvs_set - own_remote_srvs_set
            if srv2add:
                for s in srv2add:
                    if not (s in self.__servicelist):
                        srvs_added.add(s)
                        self.__servicelist[s] = other_local_srvs[s].copy(self.masteruri)

        return (nodes_added, nodes_changed, nodes2remove, topics_added, topics_changed, topics_removed, srvs_added, services_changed, srvs2remove)
