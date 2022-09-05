from typing import List
from rclpy.node import Node

from fkie_multimaster_msgs.parameters.ros2_parameter_interface import ParameterInterface
from fkie_multimaster_msgs.crossbar.runtime_interface import RosParameter
from fkie_multimaster_msgs.logging.logging import Log


class ROS2Parameters:
    def __init__(self, node: Node) -> None:
        self.interface = ParameterInterface(node)

    def getParameterList(self) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types
        '''
        param_list: List[RosParameter] = []
        try:
            param_list = self.interface.list()
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Error - {e}')
            return param_list
        return param_list

    def getNodeParameters(self, nodes: List[str]) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        param_list: List[RosParameter] = []

        try:
            param_list = self.interface.list(nodes)
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Error - {e}')
            return param_list

        return param_list

    def hasParameter(self, parameter_name: str) -> bool:
        '''
        Check if a parameter exists
        '''
        result = False

        try:
            result = self.interface.exist(parameter_name)
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Error - {e}')
            return False

        return result

    def setParameter(self, parameter: RosParameter) -> bool:
        '''
        Set the value of a parameter
        '''
        try:
            return self.interface.set(parameter)
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::setParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False

    def deleteParameter(self, parameters: List[str]) -> bool:
        '''
        Delete a list of parameter
        '''
        try:
            overall_result = True
            for parameter in parameters:
                if not self.interface.delete(parameter):
                    overall_result = False
            return overall_result
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::deleteParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False
