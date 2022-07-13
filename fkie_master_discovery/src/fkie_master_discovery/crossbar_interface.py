import json
from json import JSONEncoder
import os

# ROS dependencies
import rosnode
import rospy
from rosgraph import Master
from typing import List, Dict


def get_namespace(name):
    '''
    :param str name: the name of the node or namespace.
    :return: The namespace of given name. The last character is always a slash.
    :rtype: str
    '''
    result = os.path.dirname(name)
    if not result.endswith(rospy.names.SEP):
        result += rospy.names.SEP
    return result


def get_node_name(name):
    '''
    :param str name: the complete name of the node.
    :return: The name without namespace.
    :rtype: str
    '''
    result = os.path.basename(name).strip(rospy.names.SEP)
    return result


class SelfEncoder(JSONEncoder):
    def default(self, obj):
        return obj.__dict__


class RosTopic:
    def __init__(self, name: str, msgtype: str) -> None:
        self.name = name
        self.msgtype = msgtype
        self.publisher: List[str] = []
        self.subscriber: List[str] = []

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class RosService:
    def __init__(self, name: str, srvtype: str) -> None:
        self.name = name
        self.srvtype = srvtype
        self.masteruri = ''
        self.service_API_URI = ''
        self.provider: List[str] = []
        self.location = 'local'

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class RosNode:
    def __init__(self, id: str, name: str) -> None:
        self.id = id
        self.name = get_node_name(name)
        self.namespace = get_namespace(name)
        self.status = 'running'
        self.pid = -1
        self.node_API_URI = ''
        self.masteruri = ''
        self.location = 'local'
        self.publishers: List[RosTopic] = []
        self.subscribers: List[RosTopic] = []
        self.services: List[RosService] = []

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class RosProvider:
    '''
    :param str name: the name of the ROS Host
    :param str host: hostname
    :param int port: port of crossbar server
    '''
    def __init__(self, name: str, host: str, port: int) -> None:
        self.name = name
        self.host = host
        self.port = port
        self.type = 'crossbar-wamp'

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)
