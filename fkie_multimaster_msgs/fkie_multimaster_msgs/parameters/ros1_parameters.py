from typing import List
import rospy

from fkie_multimaster_msgs.crossbar.runtime_interface import RosParameter
from fkie_multimaster_msgs.logging.logging import Log


class ROS1Parameters:

    def getParameterList(self) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types
        '''
        param_list: List[RosParameter] = []
        param_name_list = []

        try:
            param_name_list = rospy.get_param_names()
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Could not get param names - {e}')
            return param_list

        # get parameter values
        for param_name in param_name_list:
            param_value = rospy.get_param(param_name)
            param_list.append(RosParameter(param_name, param_value))
        return param_list

    def getNodeParameters(self, nodes: List[str]) -> List[RosParameter]:
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        param_list = []
        param_name_list = []

        try:
            param_name_list = rospy.get_param_names()
        except:
            Log.error(
                f'Error at ParameterServicer::getNodeParameters: Could not get param names for nodes {nodes}')
            return param_list

        for node_name in nodes:
            for param_name in param_name_list:
                # discard parameters that does not belong to the node
                if node_name not in param_name:
                    continue
                param_value = rospy.get_param(param_name)
                param_list.append(RosParameter(param_name, param_value))
        return param_list

    def hasParameter(self, parameter_name: str) -> bool:
        '''
        Check if a parameter exists
        '''
        return rospy.has_param(parameter_name)

    def setParameter(self, parameter: RosParameter) -> bool:
        '''
        Set the value of a parameter
        '''
        try:
            rospy.set_param(parameter.name, parameter.value)
            return True
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::setParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False

    def deleteParameter(self, parameters: List[str]) -> bool:
        '''
        Delete a list of parameter
        '''
        try:
            for parameter in parameters:
                rospy.delete_param(parameter)
            return True
        except Exception as e:
            Log.error(
                f'Error at ParameterServicer::deleteParameter: [Name: {parameter.name}, Value: {parameter.value}] - {e}')
        return False
