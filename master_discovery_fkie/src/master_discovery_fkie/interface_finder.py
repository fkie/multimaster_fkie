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
import xmlrpclib

import roslib; roslib.load_manifest('master_discovery_fkie')
import rospy


def hostFromUri(uri):
  '''
  Extracts the hostname from given uri. 

  :param uri: the uri to parse

  :type uri:  str

  :return: the hostname or `None`, if the uri is `None` or `invalid`

  :rtype: str

  :see: http://docs.python.org/library/urlparse.html

  '''
  if uri is None:
    return None
  from urlparse import urlparse
  try:
    o = urlparse(uri)
    return o.hostname
  except:
    return None


def get_changes_topic(masteruri, wait=True):
  '''
  Search in publishers of ROS master for a topic with type `master_discovery_fkie.msg.MasterState <http://www.ros.org/doc/api/master_discovery_fkie/html/msg/MasterState.html>`_ and 
  returns his name, if it runs on the local host. Returns empty list if no topic
  was found and `wait` is ``False``.

  :param masteruri: the URI of the ROS master

  :type masteruri: str

  :param wait: check every second for the topic

  :type wait: bool

  :return: the list with names of the topics of type `master_discovery_fkie.msg.MasterState <http://www.ros.org/doc/api/master_discovery_fkie/html/msg/MasterState.html>`_

  :rtype: list of strings
  '''
  return _get_topic(masteruri, 'MasterState', wait)

def get_stats_topic(masteruri, wait=True):
  '''
  Search in publishers of ROS master for a topic with type LinkStatesStamped and 
  returns his name, if it runs on the local host. Returns empty list if no topic
  was found and `wait` is ``False``.
  
  :param masteruri: the URI of the ROS master
  
  :type masteruri: str
  
  :param wait: check every second for the topic
  
  :type wait: bool
  
  :return: the list of names of the topic with type `master_discovery_fkie.msg.LinkStatesStamped <http://www.ros.org/doc/api/master_discovery_fkie/html/msg/LinkStatesStamped.html>`_

  :rtype: list of strings
  '''
  return _get_topic(masteruri, 'LinkStatesStamped', wait)

def _get_topic(masteruri, ttype, wait=True):
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

  :return: the list of names of the topic with type `master_discovery_fkie.msg.LinkStatesStamped <http://www.ros.org/doc/api/master_discovery_fkie/html/msg/LinkStatesStamped.html>`_

  :rtype: list of strings
  '''
  result = []
  while not result and not rospy.is_shutdown():
    master = xmlrpclib.ServerProxy(masteruri)
    # get the system state to resolve the published nodes
    code, _, state = master.getSystemState(rospy.get_name())
    # read topic types
    code, msg, val = master.getPublishedTopics(rospy.get_name(), '')
    if code == 1:
      # search for a topic with type MasterState
      for topic, topic_type in val:
        if topic_type.endswith(ttype):
          # get the name of the publisher node
          for t, l in state[0]:
            if topic == t:
              # get the URI of the publisher node
              for n in l:
                code, msg, val = master.lookupNode(rospy.get_name(), n)
                # only local publisher will be tacked
                if code == 1 and hostFromUri(val) == hostFromUri(masteruri):
                  result.append(topic)
      if not result and wait:
        rospy.logwarn("Master_discovery node appear not to running. Wait for topic with type '%s."%ttype)
        time.sleep(1)
    elif not result and wait:
      rospy.logwarn("Can't get published topics from ROS master: %s, %s. Will keep trying!"%(code, msg))
      time.sleep(1)
    if not wait:
      return result
  return result


def get_listmaster_service(masteruri, wait=True):
  '''
  Search in services of ROS master for a service with name ending by 
  `list_masters` and returns his name, if it runs on the local host. Returns 
  empty list if no service was found and `wait` is ``False``.

  :param masteruri: the URI of the ROS master

  :type masteruri: str

  :param wait: check every second for the service

  :type wait: boo

  :return: the list with names of the services ending with `list_masters`

  :rtype: list of strings
  '''
  return _get_service(masteruri, 'list_masters', wait)

def get_refresh_service(masteruri, wait=True):
  '''
  Search in services of ROS master for a service with name ending by 
  `refresh` and returns his name, if it runs on the local host. Returns 
  empty list if no service was found and `wait` is ``False``.

  :param masteruri: the URI of the ROS master

  :type masteruri: str

  :param wait: check every second for the service

  :type wait: boo

  :return: the list with names of the services ending with `refresh`

  :rtype: list of strings
  '''
  return _get_service(masteruri, 'refresh', wait)

def _get_service(masteruri, name, wait=True):
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

  :return: the list with names of the services ending with `refresh`

  :rtype: list of strings
  '''
  result = []
  while not result and not rospy.is_shutdown():
    master = xmlrpclib.ServerProxy(masteruri)
    code, msg, val = master.getSystemState(rospy.get_name())
    if code == 1:
      pubs, subs, srvs = val
      # search for a service
      for srv, providers in srvs:
        if srv.endswith(name):
          # only local service will be tacked
          code, msg, val = master.lookupService(rospy.get_name(), srv)
          if code == 1 and hostFromUri(val) == hostFromUri(masteruri):
            result.append(srv)
      if not result and wait:
        rospy.logwarn("Master_discovery node appear not to running. Wait for service '%s'."%name)
        time.sleep(1)
    elif not result and wait:
      rospy.logwarn("can't get state from ROS master: %s, %s"%(code, msg))
      time.sleep(1)
    if not wait:
      return result
  return result
