from typing import List
import json
import asyncio
from autobahn import wamp
from types import SimpleNamespace

from fkie_multimaster_pylib.crossbar.runtime_interface import RosParameter
from fkie_multimaster_pylib.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_pylib.crossbar.base_session import SelfEncoder
from fkie_multimaster_pylib.logging.logging import Log
from fkie_multimaster_pylib.parameters.ros2_parameters import ROS2Parameters
import fkie_node_manager_daemon as nmd


class ParameterServicer(CrossbarBaseSession):
    '''
    Interface for ROS2 parameters
    '''

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11811, test_env=False) -> None:
        Log.info("Create ROS2 parameter servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port, test_env=test_env)
        self._handler = ROS2Parameters(nmd.ros_node)

    def stop(self):
        self.shutdown()

    @wamp.register('ros.parameters.get_list')
    def getParameterList(self):
        '''
        Return a list with all registered parameters values and types
        '''
        p_list = []
        Log.info(
            'ros.parameters.get_list: Getting parameters for all nodes')
        p_list = self._handler.getParameterList()
        return json.dumps(p_list, cls=SelfEncoder)

    @wamp.register('ros.parameters.get_node_parameters')
    def getNodeParameters(self, nodes: List[str]):
        '''
        Return a list with all registered parameters values and types for a given Node
        '''
        p_list = []
        Log.info(
            f'ros.parameters.get_node_parameters: Getting parameters for nodes: [{nodes}] ')
        p_list = self._handler.getNodeParameters(nodes)
        return json.dumps(p_list, cls=SelfEncoder)

    @wamp.register('ros.parameters.has_parameter')
    def hasParameter(self, parameter_name: str):
        '''
        Check if a parameter exists
        '''
        result = False
        Log.info(
            f'ros.parameters.has_parameter: Checking parameter [{parameter_name}] ')
        result = self._handler.hasParameter(parameter_name)
        return json.dumps(result, cls=SelfEncoder)

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
        result = self._handler.setParameter(parameter)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.parameters.delete_parameters')
    def deleteParameters(self, parameters: List[str]):
        '''
        Delete a list of parameters
        '''
        result = False
        Log.info(
            f'ros.parameters.delete_parameters: Removing [{parameters}] ')
        result = self._handler.deleteParameter(parameters)
        return json.dumps(result, cls=SelfEncoder)
