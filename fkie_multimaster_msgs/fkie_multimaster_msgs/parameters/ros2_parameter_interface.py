from typing import List
import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names
from ros2param.api import call_describe_parameters, call_get_parameters, get_parameter_value, call_set_parameters
from ros2service.api import get_service_names
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue

from fkie_multimaster_msgs.crossbar.runtime_interface import RosParameter
from fkie_multimaster_msgs.logging.logging import Log


class ParameterInterface:
    """ROS2 Parameter interface"""

    def __init__(self, global_node: Node) -> None:
        self.include_hidden_nodes = True  # Consider hidden nodes as well
        self.param_prefixes = []  # Only list parameters with the provided prefixes
        self.global_node = global_node

    def list(self, nodes: List[str] = None) -> List[RosParameter]:
        param_list: List[RosParameter] = []

        # get nodes and services
        node_names = get_node_names(
            node=self.global_node, include_hidden_nodes=self.include_hidden_nodes)

        service_names = get_service_names(
            node=self.global_node, include_hidden_services=self.include_hidden_nodes)

        clients = {}
        futures = {}

        # create clients for nodes which have the service
        for node_name in node_names:
            if nodes is not None and node_name.full_name not in nodes:
                continue

            service_name = f'{node_name.full_name}/list_parameters'
            if service_name in service_names:
                client = self.global_node.create_client(ListParameters, service_name)
                clients[node_name] = client

        # wait until all clients have been called
        while True:
            for node_name in [
                n for n in clients.keys() if n not in futures
            ]:
                # call as soon as ready
                client = clients[node_name]
                if client.service_is_ready():
                    request = ListParameters.Request()
                    for prefix in self.param_prefixes:
                        request.prefixes.append(prefix)
                    future = client.call_async(request)
                    futures[node_name] = future

            if len(futures) == len(clients):
                break
            rclpy.spin_once(self.global_node, timeout_sec=1.0)

        # wait for all responses
        for future in futures.values():
            rclpy.spin_until_future_complete(self.global_node, future, timeout_sec=1.0)

        # parse responses
        for node_name in sorted(futures.keys()):
            future = futures[node_name]
            if future.result() is None:
                e = future.exception()
                Log.error('Exception while calling service of node '
                          f"'{node_name.full_name}': {e}")
                continue

            response = future.result()
            sorted_names = sorted(response.result.names)

            # # get descriptors for the node, needed to print parameter type
            # name_to_type_map = {}
            # resp = call_describe_parameters(
            #     node=self.global_node, node_name=node_name.full_name,
            #     parameter_names=sorted_names)
            # for descriptor in resp.descriptors:
            #     name_to_type_map[descriptor.name] = get_parameter_type_string(
            #         descriptor.type)

            # for name in sorted_names:
            #     param_type_str = name_to_type_map[name]
            #     # print(f'  {name} (type: {param_type_str})')

            # get parameter values
            resp = call_get_parameters(
                node=self.global_node,
                node_name=node_name.full_name,
                parameter_names=sorted_names)

            for (index, parameter) in enumerate(resp.values):
                param_name = f'{node_name.full_name}/{sorted_names[index]}'
                param_list.append(
                    RosParameter(param_name, self._get_value(parameter), self._get_type(parameter)))

        return param_list

    def exist(self, parameter_name: str):
        node_name = self._get_node_name(parameter_name)
        if node_name is None:
            return False

        param_list = self.list([node_name])

        # search parameter on nodes's registered parameters
        for p in param_list:
            if p.name == parameter_name:
                return True

        return False

    def set(self, _parameter: RosParameter) -> bool:
        node_name = self._get_node_name(_parameter.name)
        if node_name is None:
            return False

        parameter = Parameter()
        parameter.name = _parameter.name.replace(f'{node_name}/', '')
        parameter.value = get_parameter_value(string_value=f'{_parameter.value}')

        response = call_set_parameters(
            node=self.global_node, node_name=node_name, parameters=[parameter])

        # output response
        if len(response.results) == 0:
            return False

        result = response.results[0]
        if result.successful:
            return True

        Log.error(f'Setting parameter failed: ', parameter, result.reason)
        return False

    def delete(self, parameter_name: str):
        node_name = self._get_node_name(parameter_name)
        if node_name is None:
            overall_result = False
            Log.error(f'Deleting parameter failed: ', parameter, "Node name not found")
            return False

        parameter = Parameter()
        parameter.name = parameter_name.replace(f'{node_name}/', '')
        value = ParameterValue()
        value.type = ParameterType.PARAMETER_NOT_SET
        parameter.value = value

        response = call_set_parameters(
            node=self.global_node, node_name=node_name, parameters=[parameter])

        # output response
        if len(response.results) == 0:
            overall_result = False
            Log.error(f'Deleting parameter failed: ', parameter, "Empty result")
            return False

        result = response.results[0]
        if result.successful:
            return True

        Log.error(f'Deleting parameter failed: ', parameter, result.reason)
        return False

    def _get_value(self, parameter_value):
        """Get the value from a ParameterValue."""
        if parameter_value.type == ParameterType.PARAMETER_BOOL:
            value = parameter_value.bool_value
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
            value = parameter_value.integer_value
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
            value = parameter_value.double_value
        elif parameter_value.type == ParameterType.PARAMETER_STRING:
            value = parameter_value.string_value
        elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            value = list(parameter_value.byte_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            value = list(parameter_value.bool_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            value = list(parameter_value.integer_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            value = list(parameter_value.double_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            value = list(parameter_value.string_array_value)
        elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            value = None

        return value

    def _get_type(self, parameter_value):
        """Get the value from a ParameterValue."""
        if parameter_value.type == ParameterType.PARAMETER_BOOL:
            return "bool"
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
            return "int"
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
            return "float"
        elif parameter_value.type == ParameterType.PARAMETER_STRING:
            return "str"
        elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return "array"
        elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return "array"
        elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return "array"
        elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return "array"
        elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return "array"
        elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            value = None

        return value

    def _get_node_name(self, parameter_name: str):
        # get node name and parameter name
        p_split = parameter_name.split("/")
        if len(p_split) == 0:
            return None

        # TODO: Fix qos_overrides parameters

        param_name = p_split.pop()
        node_name = parameter_name.replace(f'/{param_name}', "")
        return node_name
