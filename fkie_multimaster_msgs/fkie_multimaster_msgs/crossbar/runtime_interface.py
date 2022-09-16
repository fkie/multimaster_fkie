import json
import os
import platform
from typing import List, Dict, Union
import re

SEP = '/'
try:
    import rospy
    SEP = rospy.names.SEP
except ImportError:
    pass


def get_namespace(name):
    '''
    :param str name: the name of the node or namespace.
    :return: The namespace of given name. The last character is always a slash.
    :rtype: str
    '''
    result = os.path.dirname(name)
    # if not result.endswith(SEP):
    #     result += SEP
    return result


def get_node_name(name):
    '''
    :param str name: the complete name of the node.
    :return: The name without namespace.
    :rtype: str
    '''
    result = os.path.basename(name).strip(SEP)
    return result


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


class RosParameter:
    '''
    Models a ROS parameter object
    '''

    def __init__(self, name: str, value: Union[float, bool, str, List, Dict], type: str = None) -> None:
        self.name = name
        self.value = value
        self.type = type

        if self.type is None:
            self.type = self.get_type()

    def get_type(self):
        '''
        Return object type as string: for instance, from [<class 'int'>]  to "int"
        '''
        if(self.type is not None):
            return self.type

        # Try to infer type based on value
        return re.findall("'(.*)'", str(type(self.value)))[0]

    def __str__(self) -> str:
        return f'{self.name}: {self.value} ({self.type})'

    def __repr__(self):
        return f'{self.name}: {self.value} ({self.type})'


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
        self.screens: List[str] = []
        self.parameters: List[RosParameter] = []

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class RosProvider:
    '''
    :param str name: the name of the ROS Host
    :param str host: hostname
    :param int port: port of crossbar server
    :param str masteruri: master uri
    '''

    def __init__(self, name: str, host: str, port: int, masteruri: str = '') -> None:
        # Add ROS and system information
        self.ros_version = os.environ['ROS_VERSION']
        self.ros_distro = os.environ['ROS_DISTRO']
        self.ros_domain_id = os.environ['ROS_DOMAIN_ID']
        self.platform = f'{platform.system()}'
        self.platform_details = f'{platform.version()} {platform.machine()}'

        # add distro to name, to prevent collisions when ROS1 and ROS2
        # run simultaneously on the same host
        self.name = f'{name} [{self.ros_distro}]'
        self.host = host
        self.port = port
        self.type = 'crossbar-wamp'
        self.masteruri = masteruri

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class ScreenRepetitions:
    '''
    :param str name: full node name
    :param [str] screens: list the screen names associated with given node.
    '''

    def __init__(self, name: str, screens: List[str]) -> None:
        self.name = name
        self.screens = screens

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)
