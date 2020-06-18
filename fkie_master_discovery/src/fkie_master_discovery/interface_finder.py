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
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

import rospy
from .common import get_hostname


def get_changes_topic(masteruri, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with type `fkie_master_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/MasterState.html>`_ and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the topics of type `fkie_master_discovery.msg.MasterState <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/MasterState.html>`_

    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'MasterState', wait, check_host)


def get_stats_topic(masteruri, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with type LinkStatesStamped and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list of names of the topic with type `fkie_master_discovery.msg.LinkStatesStamped <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/LinkStatesStamped.html>`_

    :rtype: list of strings
    '''
    return _get_topic(masteruri, 'LinkStatesStamped', wait, check_host)


def _get_topic(masteruri, ttype, wait=True, check_host=True):
    '''
    Search in publishers of ROS master for a topic with given type and
    returns his name, if it runs on the local host. Returns empty list if no topic
    was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param ttype: the type of the topic

    :type ttype: str

    :param wait: check every second for the topic

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list of names of the topic with type `fkie_master_discovery.msg.LinkStatesStamped <http://www.ros.org/doc/api/fkie_master_discovery/html/msg/LinkStatesStamped.html>`_

    :rtype: list of strings
    '''
    result = []
    while not result and not rospy.is_shutdown():
        master = xmlrpcclient.ServerProxy(masteruri)
        # get the system state to resolve the published nodes
        code, _, state = master.getSystemState(rospy.get_name())
        # read topic types
        code, msg, val = master.getPublishedTopics(rospy.get_name(), '')
        if code == 1:
            own_host = get_hostname(masteruri)
            nodes_host = []
            # search for a topic with type MasterState
            for topic, topic_type in val:
                if topic_type.endswith(ttype):
                    # get the name of the publisher node
                    for t, l in state[0]:
                        if topic == t:
                            if check_host:
                                # get the URI of the publisher node
                                for n in l:
                                    code, msg, val = master.lookupNode(rospy.get_name(), n)
                                    # only local publisher will be tacked
                                    if code == 1:
                                        hode_host = get_hostname(val)
                                        if hode_host == own_host:
                                            result.append(topic)
                                        else:
                                            nodes_host.append(hode_host)
                            else:
                                result.append(topic)
            if not result and wait:
                rospy.logwarn("master_discovery node appear not to running @%s, only found on %s. Wait for topic with type '%s' @%s." % (own_host, nodes_host, ttype, own_host))
                time.sleep(1)
        elif not result and wait:
            rospy.logwarn("Can't get published topics from ROS master: %s, %s. Will keep trying!" % (code, msg))
            time.sleep(1)
        if not wait:
            return result
    return result


def get_listmaster_service(masteruri, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    `list_masters` and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the service

    :type wait: boo

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `list_masters`

    :rtype: list of strings
    '''
    return _get_service(masteruri, 'list_masters', wait, check_host)


def get_refresh_service(masteruri, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    `refresh` and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param wait: check every second for the service

    :type wait: boo

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `refresh`

    :rtype: list of strings
    '''
    return _get_service(masteruri, 'refresh', wait, check_host)


def _get_service(masteruri, name, wait=True, check_host=True):
    '''
    Search in services of ROS master for a service with name ending by
    given name and returns his name, if it runs on the local host. Returns
    empty list if no service was found and `wait` is ``False``.

    :param masteruri: the URI of the ROS master

    :type masteruri: str

    :param name: the ending name of the service

    :type name: str

    :param wait: check every second for the service

    :type wait: bool

    :param check_host: check for eqaul hostname of topic provider and master uri.

    :type check_host: bool

    :return: the list with names of the services ending with `refresh`

    :rtype: list of strings
    '''
    result = []
    while not result and not rospy.is_shutdown():
        master = xmlrpcclient.ServerProxy(masteruri)
        code, msg, val = master.getSystemState(rospy.get_name())
        if code == 1:
            pubs, subs, srvs = val
            own_host = get_hostname(masteruri)
            nodes_host = []
            # search for a service
            for srv, providers in srvs:
                if srv.endswith(name):
                    # only local service will be tacked
                    if check_host:
                        code, msg, val = master.lookupService(rospy.get_name(), srv)
                        if code == 1:
                            hode_host = get_hostname(val)
                            if hode_host == own_host:
                                result.append(srv)
                            else:
                                nodes_host.append(hode_host)
                    else:
                        result.append(srv)
            if not result and wait:
                rospy.logwarn("master_discovery node appear not to running @%s, only found on %s. Wait for service '%s' @%s." % (own_host, nodes_host, name, own_host))
                time.sleep(1)
        elif not result and wait:
            rospy.logwarn("can't get state from ROS master: %s, %s" % (code, msg))
            time.sleep(1)
        if not wait:
            return result
    return result
