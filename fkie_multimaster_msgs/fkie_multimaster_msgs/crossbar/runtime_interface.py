import json
import os
import platform
from typing import List, Dict, Union
import re
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs import names
from fkie_multimaster_msgs import ROS_VERSION

SEP = '/'
if ROS_VERSION == 1:
    import rospy
    SEP = rospy.names.SEP


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
        if (self.type is not None):
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
        self.parent_id = ''
        self.name = get_node_name(name)
        self.namespace = names.namespace(name, with_sep_suffix=False)
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
        try:
            self.ros_version = os.environ['ROS_VERSION'] if 'ROS_VERSION' in os.environ else ''
            self.ros_distro = os.environ['ROS_DISTRO'] if 'ROS_DISTRO' in os.environ else ''
            self.ros_domain_id = os.environ['ROS_DOMAIN_ID'] if 'ROS_DOMAIN_ID' in os.environ else ''
            self.platform = f'{platform.system()}'
            self.platform_details = f'{platform.version()} {platform.machine()}'
        except:
            import traceback
            Log.error(f'Error when initializing new provider [{name}]: {traceback.format_exc()}')

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


class SystemWarning:
    '''
    :param str msg: short warning message.
    :param str details: long description.
    :param str hint: note on the possible solution.
    '''
    def __init__(self, msg: str, details: str='', hint: str='') -> None:
        self.msg = msg
        self.details = details
        self.hint = hint


class SystemWarningGroup:

    ID_ADDR_MISMATCH='ADDR_MISMATCH'
    ID_RESOLVE_FAILED='RESOLVE_FAILED'
    ID_UDP_SEND='UDP_SEND'
    ID_EXCEPTION='EXCEPTION'
    ID_TIME_JUMP='TIME_JUMP'

    '''
    :param str id: id of the warning group, on of ID_*.
    :param list[SystemWarning] warnings: list of warnings.
    '''
    def __init__(self, id: str, warnings: List[SystemWarning]=None) -> None:
        self.id = id
        self.warnings = [] if warnings is None else warnings

    def append(self, warning: SystemWarning):
        self.warnings.append(warning)

    def __eq__(self, other) -> bool:
        if self.id != other.id:
            return False
        if len(self.warnings) != len(other.warnings):
            return False
        for my_warning in self.warnings:
            found = False
            for other_warning in other.warnings:
                if my_warning.msg == other_warning.msg:
                    found = True
                    break
            if not found:
                return False
        return True
