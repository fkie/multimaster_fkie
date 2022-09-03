from typing import List
import json
import re
import os
import rospy
import asyncio
from autobahn import wamp
from types import SimpleNamespace

from fkie_multimaster_msgs.crossbar.runtime_interface import RosParameter
from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.logging.logging import Log


class ParameterServicer(CrossbarBaseSession):
    '''
    Interface with the ROS parameter server ROS1
    TODO: Include also ROS2 parameters
    '''

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911, test_env=False) -> None:
        Log.info("Create parameter servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env)
        self.ros_version = os.environ['ROS_VERSION']

        if(self.ros_version == "1"):
            self._rospy = __import__('rospy')
        if(self.ros_version == "2"):
            self._rclpy = __import__('rclpy')

    @wamp.register('ros.parameters.get_list')
    def getParameterList(self):
        '''
        Return a list with all registered parameters values and types
        '''
        p_list = []

        Log.info(
            f'ros.parameters.get_list: Getting parameters for all nodes')

        if(self.ros_version == "1"):
            p_list = self._ros1GetParameterList()
        if(self.ros_version == "2"):
            p_list = self._ros2GetParameterList()
        return json.dumps(p_list, cls=SelfEncoder)

    def _ros1GetParameterList(self) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types
        '''
        param_list = []
        param_name_list = []

        try:
            param_name_list = self._rospy.get_param_names()
        except:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Could not get param names')
            return param_list

        # get parameter values
        for param_name in param_name_list:
            param_value = self._rospy.get_param(param_name)
            param_list.append(RosParameter(param_name, param_value))
        return param_list

    def _ros2GetParameterList(self) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types
        '''
        param_list = []
        param_name_list = []

        try:
            param_name_list = self._rospy.get_param_names()
        except:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Could not get param names')
            return param_list

        # get parameter values
        for param_name in param_name_list:
            param_value = self._rospy.get_param(param_name)
            param_list.append(RosParameter(param_name, param_value))
        return param_list

    @wamp.register('ros.parameters.get_node_parameters')
    def getNodeParameters(self, nodes: List[str]):
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        p_list = []

        Log.info(
            f'ros.parameters.get_node_parameters: Getting parameters for nodes: [{nodes}] ')

        if(self.ros_version == "1"):
            p_list = self._ros1GetNodeParameters(nodes)
        if(self.ros_version == "2"):
            p_list = self._ros2GetNodeParameters(nodes)

        return json.dumps(p_list, cls=SelfEncoder)

    def _ros1GetNodeParameters(self, nodes: List[str]) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        param_list = []
        param_name_list = []

        try:
            param_name_list = self._rospy.get_param_names()
        except:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Could not get param names for nodes {nodes}')
            return param_list

        for node_name in nodes:
            for param_name in param_name_list:
                # discard parameters that does not belong to the node
                if node_name not in param_name:
                    continue
                param_value = self._rospy.get_param(param_name)
                param_list.append(RosParameter(param_name, param_value))
        return param_list

    def _ros2GetNodeParameters(self, nodes: List[str]) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        param_list = []
        param_name_list = []

        try:
            param_name_list = self._rospy.get_param_names()
        except:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Could not get param names for nodes {nodes}')
            return param_list

        for node_name in nodes:
            for param_name in param_name_list:
                # discard parameters that does not belong to the node
                if node_name not in param_name:
                    continue
                param_value = self._rospy.get_param(param_name)
                param_list.append(RosParameter(param_name, param_value))
        return param_list

    @wamp.register('ros.parameters.has_parameter')
    def hasParameter(self, parameter_name: str):
        '''
        Check if a parameter exists
        '''
        result = False

        Log.info(
            f'ros.parameters.has_parameter: Checking parameter [{parameter_name}] ')

        if(self.ros_version == "1"):
            result = self._ros1HasParameter(parameter_name)
        if(self.ros_version == "2"):
            result = self._ros2HasParameter(parameter_name)

        return json.dumps(result, cls=SelfEncoder)

    def _ros1HasParameter(self, parameter_name: str) -> bool:
        '''
        Check if a parameter exists
        '''
        return self._rospy.has_param(parameter_name)

    def _ros2HasParameter(self, parameter_name: str) -> bool:
        '''
        Check if a parameter exists
        '''
        return self._rospy.has_param(parameter_name)

    @wamp.register('ros.parameters.set_parameter')
    def setParameter(self, _parameter: RosParameter):
        '''
        Set the value of a parameter
        '''
        parameter = json.loads(json.dumps(_parameter),
                               object_hook=lambda d: SimpleNamespace(**d))
        Log.info(
            f'ros.parameters.set_parameter: [{parameter.name}] to {parameter.value}')
        result = None
        if(self.ros_version == "1"):
            result = self._ros1SetParameter(parameter)
        if(self.ros_version == "2"):
            result = self._ros2SetParameter(parameter)

        return json.dumps(result, cls=SelfEncoder)

    def _ros1SetParameter(self, parameter: RosParameter) -> bool:
        '''
        Set the value of a parameter
        '''
        try:
            self._rospy.set_param(parameter.name, parameter.value)
            return True
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::setParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False

    def _ros2SetParameter(self, parameter: RosParameter) -> bool:
        '''
        Set the value of a parameter
        '''
        try:
            self._rospy.set_param(parameter.name, parameter.value)
            return True
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::setParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False

    @wamp.register('ros.parameters.delete_parameters')
    def deleteParameters(self, parameters: List[str]):
        '''
        Delete a list of parameters
        '''
        result = False
        Log.info(
            f'ros.parameters.delete_parameters: Removing [{parameters}] ')

        if(self.ros_version == "1"):
            result = self._ros1DeleteParameter(parameters)
        if(self.ros_version == "2"):
            result = self._ros2DeleteParameter(parameters)

        return json.dumps(result, cls=SelfEncoder)

    def _ros1DeleteParameter(self, parameters: List[str]) -> bool:
        '''
        Delete a list of parameter
        '''
        try:
            for parameter in parameters:
                self._rospy.delete_param(parameter)
            return True
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::deleteParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False

    def _ros2DeleteParameter(self, parameters: List[str]) -> bool:
        '''
        Delete a list of parameter
        '''
        try:
            for parameter in parameters:
                self._rospy.delete_param(parameter)
            return True
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::deleteParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False
