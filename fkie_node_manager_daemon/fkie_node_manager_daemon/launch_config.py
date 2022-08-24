# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List
from typing import Text
from typing import Tuple

from xml.dom.minidom import parse  # , parseString
import os
import re
import sys
import time

import launch
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch.frontend.parser import Parser
from launch.launch_description_sources import get_launch_description_from_frontend_launch_file
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch.utilities import normalize_to_list_of_substitutions
import launch_ros
import rclpy

from fkie_node_manager_daemon.grpc_proto.launch_pb2 import Argument  # pylint: disable=no-name-in-module, import-error

import fkie_node_manager_daemon as nmd

from .common import SEP
from .common import package_name
from .common import ns_join
from .launch_context import LaunchContext
from .url import nmduri as url_nmduri


class LaunchConfigException(Exception):
    pass


class LaunchNode():

    def __init__(self, node, composable_container=None, composable_nodes=[]):
        self.node = node
        self.composable_container = composable_container
        # create a new array to a void to fill a default one
        self.composable_nodes = composable_nodes if composable_nodes else []


class LaunchConfig(object):
    '''
    A class to handle the ROS configuration stored in launch file.
    '''

    def __init__(self, launch_file, *, context=None, package=None, daemonuri='', launch_arguments: List[Tuple[Text, Text]] = []):
        '''
        Creates the LaunchConfig object. The launch file will be not loaded on
        creation, first on request of roscfg value.

        :param str launch_file: The absolute or relative path with the launch file.
                                By using relative path a package must be valid for
                                remote launches.
        :param package: the package containing the launch file. If None the
                        launch_file will be used to determine the launch file.
                        No remote launches a possible without a valid package.
        :type package: str or None
        :param str daemonuri: daemon where to start the nodes of this launch file.
        :raise roslaunch.XmlParseException: if the launch file can't be found.
        '''
        self.__launchfile = launch_file
        self.__package = package_name(os.path.dirname(self.__launchfile))[
            0] if package is None else package
        self.__nmduri = url_nmduri(daemonuri)
        self.launch_arguments = launch_arguments
        argv = sys.argv[1:]
        argv.extend(["%s:=%s" % (name, value)
                     for (name, value) in launch_arguments])
        self.__launch_context = context if context is not None else LaunchContext(
            argv=argv)
        self.__launch_description = get_launch_description_from_any_launch_file(
            self.filename)
        # self.__launch_description = launch.LaunchDescription([
        #     launch.actions.IncludeLaunchDescription(
        #         launch.launch_description_sources.AnyLaunchDescriptionSource(
        #             self.filename
        #         ),
        #         launch_arguments=launch_arguments,
        #     ),
        # ])
        # for ent in self.__launch_description.entities:
        #    ent.execute(self.__launch_context)
        print("LD", dir(self.__launch_description))
        self._load()
        self.argv = None
        if self.argv is None:
            self.argv = []
        self.__reqTested = False
        self.__argv_values = dict()
        self.__launch_id = '%.9f' % time.time()
        self._robot_description = None
        self._capabilities = None
        self.resolve_dict = {}
        self.changed = True

#     def __del__(self):
#         pass

    @property
    def context(self) -> LaunchContext:
        return self.__launch_context

    @property
    def daemonuri(self) -> str:
        '''
        :return: Returns the URI (host) of daemon where the node of this config will be started.
        :rtype: str
        '''
        return self.__nmduri

    @property
    def roscfg(self):
        '''
        Holds a loaded launch configuration. It raises a LaunchConfigException on load error.

        :return: a previously loaded ROS configuration
        :rtype: :meth:`roslaunch.ROSLaunchConfig` <http://docs.ros.org/kinetic/api/roslaunch/html/> or None
        :any: :meth:`load`
        '''
        if self.__launch_description is not None:
            return self.__launch_description
        else:
            result, _ = self.load(self.argv)  # _:=argv
            if not result:
                raise LaunchConfigException(
                    "not all argv are setted properly!")
            return self.__launch_description

    def _load(self, sub_obj=None) -> None:
        if sub_obj is None:
            sub_obj = self.__launch_description
        entities = None
        if hasattr(sub_obj, 'get_sub_entities'):
            entities = getattr(sub_obj, 'get_sub_entities')()
        elif hasattr(sub_obj, 'entities'):
            entities = getattr(sub_obj, 'entities')
        if entities is not None:
            for entity in entities:
                if isinstance(entity, launch_ros.actions.node.Node):
                    # for cmds in entity.cmd:
                    #     for cmd in cmds:
                    #         if isinstance(cmd, launch_ros.substitutions.executable_in_package.ExecutableInPackage):
                    #             print('  - CMD InExc:', cmd.describe(), dir(cmd.describe))
                    #             print('      + CMD exe:', cmd.executable[0].text)
                    #             print('      + CMD package:', cmd.package[0].text)
                    #             print('      + perform:', cmd.perform(self.__launch_context))
                    #         elif isinstance(cmd, launch.substitutions.text_substitution.TextSubstitution):
                    #             print('      + CMD:', cmd.text, dir(cmd))
                    #             print('      + perform:', cmd.perform(self.__launch_context))
                    #         elif isinstance(cmd, launch.actions.pop_launch_configurations.PopLaunchConfigurations):
                    #             print('      + CMD:', cmd.describe(), dir(cmd))
                    #         elif isinstance(cmd, launch.substitutions.local_substitution.LocalSubstitution):
                    #             print('      + CMD Subst:', cmd.expression, dir(cmd.expression))
                    #             # print('      + perform:', cmd.perform(self.__launch_context))
                    #         else:
                    #             print('      + CMD OTHER:', cmd, dir(cmd))
                    entity._perform_substitutions(self.context)
                    if entity.expanded_node_namespace == launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE:
                        entity.__expanded_node_namespace = ''
                elif isinstance(entity, launch.actions.execute_process.ExecuteProcess):
                    print("EXEC", type(entity), dir(entity))
                    # entity._perform_substitutions(self.context)
                elif isinstance(entity, launch.actions.declare_launch_argument.DeclareLaunchArgument):
                    print('  perform ARG:', entity.name)
                    entity.execute(self.__launch_context)
                elif isinstance(entity, launch.actions.include_launch_description.IncludeLaunchDescription):
                    try:
                        entity.execute(self.__launch_context)
                    except launch.invalid_launch_file_error.InvalidLaunchFileError as err:
                        print('err', dir(err))
                        print('launch_description_source', dir(
                            entity.launch_description_source.location))
                        raise Exception('%s (%s)' % (
                            err, entity.launch_description_source.location))
                elif hasattr(entity, 'execute'):
                    print("TY2", type(entity), dir(entity))
                    entity.execute(self.__launch_context)
                self._load(entity)

    def nodes(self, sub_obj=None) -> List[LaunchNode]:
        result = []
        if sub_obj is None:
            sub_obj = self.__launch_description
        entities = None
        if hasattr(sub_obj, 'get_sub_entities'):
            entities = getattr(sub_obj, 'get_sub_entities')()
        elif hasattr(sub_obj, 'entities'):
            entities = getattr(sub_obj, 'entities')
        if entities is not None:
            for entity in entities:
                if isinstance(entity, launch_ros.actions.composable_node_container.ComposableNodeContainer):
                    container = LaunchNode(entity)
                    cnodes = []
                    for cn in entity._ComposableNodeContainer__composable_node_descriptions:
                        node = LaunchNode(cn, entity)
                        container.composable_nodes.append(cn)
                        cnodes.append(node)
                    result.append(container)
                    result.extend(cnodes)
                elif isinstance(entity, launch_ros.actions.node.Node):
                    # for cmds in entity.cmd:
                    #     for cmd in cmds:
                    #         if isinstance(cmd, launch_ros.substitutions.executable_in_package.ExecutableInPackage):
                    #             print('  - CMD InExc:', cmd.describe(), dir(cmd.describe))
                    #             print('      + CMD exe:', cmd.executable[0].text)
                    #             print('      + CMD package:', cmd.package[0].text)
                    #             print('      + perform:', cmd.perform(self.__launch_context))
                    #         elif isinstance(cmd, launch.substitutions.text_substitution.TextSubstitution):
                    #             print('      + CMD:', cmd.text, dir(cmd))
                    #             print('      + perform:', cmd.perform(self.__launch_context))
                    #         elif isinstance(cmd, launch.actions.pop_launch_configurations.PopLaunchConfigurations):
                    #             print('      + CMD:', cmd.describe(), dir(cmd))
                    #         elif isinstance(cmd, launch.substitutions.local_substitution.LocalSubstitution):
                    #             print('      + CMD Subst:', cmd.expression, dir(cmd.expression))
                    #             # print('      + perform:', cmd.perform(self.__launch_context))
                    #         else:
                    #             print('      + CMD OTHER:', cmd, dir(cmd))
                    #                    entity._perform_substitutions(self.context)
                    result.append(LaunchNode(entity))
                elif isinstance(entity, launch.actions.execute_process.ExecuteProcess):
                    print("EXEC", type(entity), dir(entity))
                    # entity._perform_substitutions(self.context)
                    result.append(LaunchNode(entity))
                elif isinstance(entity, launch.actions.declare_launch_argument.DeclareLaunchArgument):
                    print('  perform ARG:', entity.name)
#                    entity.execute(self.__launch_context)
                else:
                    print("TY2", type(entity), dir(entity))
#                    entity.execute(self.__launch_context)
                    result.extend(self.nodes(entity))

        #     for entity in sub_obj.entities:
        #         if isinstance(entity, launch_ros.actions.node.Node):
        #             entity._perform_substitutions(self.context)
        #             result.append(entity)
        #         else:
        #             print("TY2", type(entity))
        #             result.extend(self.nodes(entity))

        # else:
        #     for entity in self.__launch_description.entities:
        #         if isinstance(entity, launch_ros.actions.node.Node):
        #             print("NODE", type(entity))
        #             subentity._perform_substitutions(self.context)
        #             result.append(entity)
        #         else:
        #             print("TY", type(entity))
        #             result.extend(self.nodes(entity))
        #             # print("INCLUDE", dir(entity), len(entity.get_sub_entities()))
                    # print("---", type(entity.get_sub_entities()[0]), dir(entity.get_sub_entities()[0]), len(entity.get_sub_entities()))
                # elif isinstance(entity, launch.actions.group_action.GroupAction):
                #     for subentity in entity.get_sub_entities():
                #         if isinstance(subentity, launch_ros.actions.node.Node):
                #             result.append(subentity)
                #         else:
                #             print("NOT launch_ros.actons.node: ", type(subentity))
                # elif isinstance(entity, launch.actions.IncludeLaunchDescription):
                #     print("INCLUDE", dir(entity))
        return result

        # for entity in self.roscfg.entities:
        #     if isinstance(entity, launch_ros.actions.node.Node):
        #         result.append(entity)
        #     elif isinstance(entity, launch.actions.group_action.GroupAction):
        #         for subentity in entity.get_sub_entities():
        #             if isinstance(subentity, launch_ros.actions.node.Node):
        #                 result.append(subentity)
        #             else:
        #                 print("NOT launch_ros.actons.node: ", type(subentity))
        #     elif isinstance(entity, launch.actions.IncludeLaunchDescription):
        #         print("INCLUDE", dir(entity))
        # return result

    @property
    def filename(self) -> Text:
        '''
        Returns an existing path with file name or an empty string.

        :rtype: str
        '''
        if os.path.isfile(self.__launchfile):
            return self.__launchfile
        elif self.packagename:
            try:
                return roslib.packages.find_resource(self.packagename, self.launchname).pop()
            except Exception:
                raise LaunchConfigException(
                    'launch file %s not found!' % self.launchname)
        raise LaunchConfigException(
            'launch file %s not found!' % self.__launchfile)

    @property
    def launchname(self):
        '''
        Returns the name of the launch file with extension, e.g. 'test.launch'

        :rtype: str
        '''
        return os.path.basename(self.__launchfile)

    @property
    def packagename(self):
        '''
        Returns the name of the package containing the launch file or None.
        :rtype: str or None
        '''
        return self.__package

    @classmethod
    def _index(cls, text, regexp_list):
        '''
        Searches in the given text for key indicates the including of a file and
        return their index.

        :param str text:
        :param regexp_list:
        :type regexp_list: list(:class:`QRegExp` <https://srinikom.github.io/pyside-docs/PySide/QtCore/QRegExp.html>})
        :return: the index of the including key or -1
        :rtype: int
        '''
        for pattern in regexp_list:
            index = pattern.indexIn(text)
            if index > -1:
                return index
        return -1

    def load(self, argv):
        '''
        :param argv: a list with argv parameter needed to load the launch file.
                     The name and value are separated by `:=`
        :type argv: list(str)
        :return: True, if the launch file was loaded and argv, used while launch
        :rtype: tuple(bool, [])
        :raise LaunchConfigException: on load errors
        '''
        try:
            self._capabilities = None
            self._robot_description = None
            print('load', self.filename)
            launch_description = get_launch_description_from_any_launch_file(
                self.filename)
            # AnyLaunchDescriptionSource(self.filename)
            # launch_description = launch.LaunchDescription([
            #     launch.actions.IncludeLaunchDescription(
            #         launch.launch_description_sources.AnyLaunchDescriptionSource(self.filename),
            #         launch_arguments=parsed_launch_arguments,
            #     ),
            # ])
            # parser = Parser.get_parser_from_extension('.launch')
            # self.__launch_context = LaunchContext(argv=sys.argv[1:])
            #launch_description = get_launch_description_from_frontend_launch_file(self.filename)
            print("roscfg", launch_description, type(
                launch_description), dir(launch_description))
#            launch_description.execute(self.__launch_context)

            for ent in launch_description.entities:
                # for cmds in ent.cmd:
                #     for cmd in cmds:
                #         print('  ', cmd, cmd.text, cmd.describe())
                if isinstance(ent, launch.actions.group_action.GroupAction):
                    for dsub_ent in ent.describe_sub_entities():
                        print("  descr_subent: ", dsub_ent, dir(dsub_ent))
                        if hasattr(dsub_ent, 'describe_sub_entities'):
                            print("    descr_subent2: ",
                                  dsub_ent.describe_sub_entities())
                            print("    get:sub_descr_subent2: ",
                                  dsub_ent.get_sub_entities())
                            for dsub_ent2 in dsub_ent.describe_sub_entities():
                                print("    descr_subent2: ",
                                      dsub_ent2, dir(dsub_ent2))
                        if hasattr(dsub_ent, 'cmd'):
                            dsub_ent._perform_substitutions(
                                self.__launch_context)
                            print('    PARAMETERS:',
                                  dsub_ent._Node__expanded_parameter_files)
                            for pp in dsub_ent._Node__expanded_parameter_files:
                                with open(pp, 'r') as f:
                                    print(f.readlines())
                                # for key, val in pp.items():
                                #     print('      param:', key[0].text, val)
                            print('    CMD:', dsub_ent.cmd)
                            for cmds in dsub_ent.cmd:
                                for cmd in cmds:
                                    if isinstance(cmd, launch_ros.substitutions.executable_in_package.ExecutableInPackage):
                                        print('  - CMD InExc:',
                                              cmd.describe(), dir(cmd.describe))
                                        print('      + CMD exe:',
                                              cmd.executable[0].text)
                                        print('      + CMD package:',
                                              cmd.package[0].text)
                                        print('      + perform:',
                                              cmd.perform(self.__launch_context))
                                    elif isinstance(cmd, launch.substitutions.text_substitution.TextSubstitution):
                                        print('      + CMD:',
                                              cmd.text, dir(cmd))
                                        print('      + perform:',
                                              cmd.perform(self.__launch_context))
                                    elif isinstance(cmd, launch.actions.pop_launch_configurations.PopLaunchConfigurations):
                                        print('      + CMD:',
                                              cmd.describe(), dir(cmd))
                                    elif isinstance(cmd, launch.substitutions.local_substitution.LocalSubstitution):
                                        print('      + CMD Subst:',
                                              cmd.expression, dir(cmd.expression))
                                        # print('      + perform:', cmd.perform(self.__launch_context))
                                    else:
                                        print('      + CMD OTHER:',
                                              cmd, dir(cmd))
                        if hasattr(dsub_ent, 'output'):
                            print('    OUTPUT:', dsub_ent.output)
                        if hasattr(dsub_ent, 'expanded_node_namespace'):
                            print('    expanded_node_namespace:',
                                  dsub_ent.expanded_node_namespace)
                        if isinstance(dsub_ent, launch.actions.declare_launch_argument.DeclareLaunchArgument):
                            print('  perform ARG:', dsub_ent.name)
                            dsub_ent.execute(self.__launch_context)
                if isinstance(ent, launch.actions.declare_launch_argument.DeclareLaunchArgument):
                    print('  perform ARG:', ent.name)
                    ent.execute(self.__launch_context)
                    # print('   context after exec:', self.__launch_context.launch_configurations[dsub_ent.name])

            # if isinstance(entity, launch_ros.actions.node.Node):
            #     print(dir(entity))
            # elif isinstance(entity, launch.actions.group_action.GroupAction):
            #     for subentity in entity.get_sub_entities():
            #         if isinstance(subentity, launch_ros.actions.node.Node):
            #             result.append(subentity)
            #         else:
            #             print(type(subentity))
            self.__launch_description = launch_description
            self.changed = True
        except launch.InvalidLaunchFileError as e:
            test = list(re.finditer(
                r"environment variable '\w+' is not set", str(e)))
            message = str(e)
            if test:
                message = '%s\nenvironment substitution is not supported, use "arg" instead!' % message
            raise LaunchConfigException(message)
        return True, self.argv

    def resolve_args(self, argv):
        argv_dict = self.argv2dict(argv)
        # replace $(arg ...) in arg values
        for k, _ in argv_dict.items():
            self._replace_arg(k, argv_dict, self.__argv_values)
        return ["%s:=%s" % (k, v) for k, v in argv_dict.items()]

    def _replace_arg(self, arg, argv_defaults, argv_values):
        '''
        Replace the arg-tags in the value in given argument recursively.
        '''
        rec_inc = 0
        value = argv_defaults[arg]
        arg_match = re.search(r"\$\(\s*arg\s*", value)
        while arg_match is not None:
            rec_inc += 1
            endIndex = value.find(')', arg_match.end())
            if endIndex > -1:
                arg_name = value[arg_match.end():endIndex].strip()
                if arg == arg_name:
                    raise LaunchConfigException(
                        "Can't resolve the argument `%s` argument: the argument referenced to itself!" % arg_name)
                if rec_inc > 100:
                    raise LaunchConfigException(
                        "Can't resolve the argument `%s` in `%s` argument: recursion depth of 100 reached!" % (arg_name, arg))
                if arg_name in argv_defaults:
                    argv_defaults[arg] = value.replace(
                        value[arg_match.start():endIndex + 1], argv_defaults[arg_name])
                elif arg_name in argv_values:
                    argv_defaults[arg] = value.replace(
                        value[arg_match.start():endIndex + 1], argv_values[arg_name])
                else:
                    raise LaunchConfigException(
                        "Can't resolve the argument `%s` in `%s` argument" % (arg_name, arg))
            else:
                raise LaunchConfigException(
                    "Can't resolve the argument in `%s` argument: `)` not found" % arg)
            value = argv_defaults[arg]
            arg_match = re.search(r"\$\(\s*arg\s*", value)

    @classmethod
    def get_args(cls, filename: str, provided_args: list) -> List[Argument]:
        '''
        :param list(fkie_node_manager_daemon.grpc_proto.launch_pb2.Argument) provided_args: provided args used to set 'value' in returned args
        :return: a list with args being used in the roslaunch file.
        :rtype: list(fkie_node_manager_daemon.grpc_proto.launch_pb2.Argument)
        '''
        launch_description = get_launch_description_from_any_launch_file(
            filename)
        launch_arguments = launch_description.get_launch_arguments()
        result = []
        for argument_action in launch_arguments:
            value = ''
            for parg in provided_args:
                if argument_action.name == parg.name:
                    value = parg.value
                    break
            default_str = ''
            if argument_action.default_value is not None:
                default_str = ' + '.join([token.describe()
                                          for token in argument_action.default_value])
            arg = Argument(name=argument_action.name,
                           value=value,
                           default_value=default_str,
                           description=argument_action.description,
                           conditionally_included=argument_action._conditionally_included)
            result.append(arg)
        return result
        # # get only the args in the top launch file
        # for filename in [self.filename]:
        #     try:
        #         if filename.endswith('.launch') or filename.endswith('.launch.xml') > 0:
        #             args[len(args):-1] = parse(filename).getElementsByTagName('arg')
        #         elif filename.endswith('.launch.py'):
        #             pass # TODO
        #     except Exception as e:
        #         raise roslaunch.XmlParseException("Invalid roslaunch XML syntax: %s" % e)
        #     for arg in args:
        #         arg_name = arg.getAttribute("name")
        #         if not arg_name:
        #             raise roslaunch.XmlParseException("arg tag needs a name, xml is %s" % arg.toxml())
        #         # we only want argsargs at top level:
        #         if not arg.parentNode.tagName == "launch":
        #             continue
        #         arg_default = arg.getAttribute("default")
        #         arg_value = arg.getAttribute("value")
        #         arg_sub = ''.join([arg_name, ':=', arg_default])
        #         if (not arg_value) and arg_sub not in arg_subs:
        #             arg_subs.append(arg_sub)
        #         elif arg_valu_decodee:
        #             self.__argv_values[arg_name] = arg_value
        # return arg_subs

    def _decode(self, val: str) -> str:
        '''
        Replaces the '\\n' by LF (Line Feed) and decode the string entry to unicode.

        :param str val: the string coding as system default
        :return: the decoded string
        :rtype: unicode or original on error
        '''
        result = val.replace("\\n ", "\n")
        try:
            result = result
        except Exception:
            pass
        return result

    def get_robot_descr(self):
        '''
        Parses the launch file for `robots` parameter to get the description of the
        robot.

        :return: the robot description stored in the configuration
        :rtype: dict(robot:dict('type' :str, 'name': str, 'images' : [str], 'description': str))
        '''
        if self._robot_description is not None:
            return self._robot_description
        self._robot_description = dict()
        if self.roscfg is not None:
            for param, p in self.roscfg.params.items():
                if param.endswith('/robots'):
                    if isinstance(p.value, list):
                        if len(p.value) > 0 and len(p.value[0]) != 5:
                            nm.rosnode.get_logger().warn(
                                "WRONG format, expected: ['host', 'type', 'name', 'images', 'description'] -> ignore; param: %s" % param)
                        else:
                            for entry in p.value:
                                self._robot_description[entry[0]] = {'type': entry[1], 'name': entry[2], 'images': entry[3].split(
                                    ','), 'description': self._decode(entry[4])}
        return self._robot_description

    def get_capabilitie_desrc(self):
        '''
        Parses the launch file for `capabilities` and `capability_group` parameter
        and creates  dictionary for grouping the nodes.

        :return: the capabilities description stored in this configuration
        :rtype: dict(machine : dict(namespace: dict(group:dict('type' : str, 'images' : [str], 'description' : str, 'nodes' : [str]))))
        '''
        if self._capabilities is not None:
            return self._capabilities
        result = dict()
        capabilies_descr = dict()
        if self.roscfg is not None:
            # get the capabilities description
            # use two separate loops, to create the description list first
            # TODO read the group description depending on namespace
            for param, p in self.roscfg.params.items():
                if param.endswith('capabilities'):
                    if isinstance(p.value, list):
                        if len(p.value) > 0 and len(p.value[0]) != 4:
                            nm.rosnode.get_logger().warn(
                                "WRONG format, expected: ['name', 'type', 'images', 'description'] -> ignore; param: %s" % param)
                        else:
                            for entry in p.value:
                                capabilies_descr[entry[0]] = {'type': '%s' % entry[1], 'images': entry[2].split(
                                    ','), 'description': self._decode(entry[3])}
            # get the capability nodes
            for item in self.roscfg.nodes:
                node_fullname = ns_join(item.namespace, item.name)
                machine_name = item.machine_name if item.machine_name is not None and not item.machine_name == 'localhost' else ''
                added = False
                cap_param = ns_join(node_fullname, 'capability_group')
                cap_ns = node_fullname
                # find the capability group parameter in namespace
                while cap_param not in self.roscfg.params and cap_param.count(SEP) > 1:
                    cap_ns = get_namespace(cap_ns).rstrip(SEP)
                    if not cap_ns:
                        cap_ns = SEP
                    cap_param = ns_join(cap_ns, 'capability_group')
                if cap_ns == node_fullname:
                    cap_ns = item.namespace.rstrip(SEP)
                    if not cap_ns:
                        cap_ns = SEP
                # if the 'capability_group' parameter found, assign node to the group
                if cap_param in self.roscfg.params and self.roscfg.params[cap_param].value:
                    p = self.roscfg.params[cap_param]
                    if machine_name not in result:
                        result[machine_name] = dict()
                    for (ns, groups) in result[machine_name].items():
                        if ns == cap_ns and p.value in groups:
                            groups[p.value]['nodes'].append(node_fullname)
                            added = True
                            break
                    if not added:
                        ns = cap_ns
                        # add new group in the namespace of the node
                        if ns not in result[machine_name]:
                            result[machine_name][ns] = dict()
                        if p.value not in result[machine_name][ns]:
                            try:
                                result[machine_name][ns][p.value] = {'type': capabilies_descr[p.value]['type'],
                                                                     'images': capabilies_descr[p.value]['images'],
                                                                     'description': capabilies_descr[p.value]['description'],
                                                                     'nodes': []}
                            except Exception:
                                result[machine_name][ns][p.value] = {'type': '',
                                                                     'images': [],
                                                                     'description': '',
                                                                     'nodes': []}
                        result[machine_name][ns][p.value]['nodes'].append(
                            node_fullname)
        self._capabilities = result
        return result

    @classmethod
    def argv2dict(cls, argv):
        result = dict()
        for a in argv:
            key, sep, value = a.partition(':=')
            if sep:
                result[key] = value
        return result

    def get_node(self, name: str, daemonuri: str = '') -> [launch_ros.actions.node.Node, None]:
        '''
        Returns a configuration node for a given node name.

        :param str name: the name of the node.
        :return: the configuration node stored in this configuration
        :rtype: :class:`launch_ros.actions.node.Node` or None
        '''
        for item in self.nodes():
            nodename = self.get_name_from_node(item.node)
            if (nodename == name):
                return item
        nmd.rosnode.get_logger().debug("Node '%s' NOT found" % name)
        return None

    @classmethod
    def get_name_from_node(cls, node: launch_ros.actions.node.Node) -> str:
        result = ''
        if hasattr(node, 'name') and node.name:
            if isinstance(node.name, str):
                result = node.name
            else:
                result = SEP.join([n.text for n in node.name])
            if result.endswith(launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAME):
                result = ''
            if result:
                nmd.rosnode.get_logger().debug("Nodename '%s' from name" % result)
        if not result and hasattr(node, 'node_name') and node.node_name:
            if isinstance(node.node_name, str):
                result = node.node_name
            else:
                result = SEP.join([n.text for n in node.node_name])
            if result.endswith(launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAME):
                result = ''
            if result:
                nmd.rosnode.get_logger().debug("Nodename '%s' from node_name" % result)
        if not result and hasattr(node, '_Node__executable'):
            # use executable
            if isinstance(node._Node__executable, str):
                result = node._Node__executable
            else:
                result = SEP.join([n.text for n in node._Node__executable])
            if result:
                nmd.rosnode.get_logger().debug("Nodename '%s' from _Node__executable" % result)
        if not result and hasattr(node, '_Node__node_executable'):
            # use node_executable; before foxy
            if isinstance(node._Node__node_executable, str):
                result = node._Node__node_executable
            else:
                result = SEP.join(
                    [n.text for n in node._Node__node_executable])
            if result:
                nmd.rosnode.get_logger().debug("Nodename '%s' from _Node__node_executable" % result)
        if not result and hasattr(node, 'cmd'):
            result = cls.cmd_to_name(node.cmd)
            if result:
                nmd.rosnode.get_logger().debug("Nodename '%s' from cmd" % result)
        if result:
            if not result.startswith(SEP) and node.expanded_node_namespace != launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE:
                ns = node.expanded_node_namespace if hasattr(
                    node, 'expanded_node_namespace') else SEP
                if not ns:
                    ns = SEP
                result = ns_join(ns, result)
        else:
            nmd.rosnode.get_logger().debug("No name for node found: %s %s" %
                                           (type(node), dir(node)))
            # print("describe", node.describe(), dir(node.describe()), str(node.describe()))
            # print("'cmd'", " ".join([str(n) for n in node.cmd]))

        return result

    @classmethod
    def cmd_to_name(cls, cmd_list):
        result = ''
        for cmds in cmd_list:
            for cmd in cmds:
                if isinstance(cmd, launch_ros.substitutions.executable_in_package.ExecutableInPackage):
                    print('  - CMD InExc:', cmd.describe(), dir(cmd.describe))
                    print('      + CMD exe:', cmd.executable[0].text)
                    print('      + CMD package:', cmd.package[0].text)
                    #print('      + perform:', cmd.perform(self.__launch_context))
                elif isinstance(cmd, launch.substitutions.text_substitution.TextSubstitution):
                    result += cmd.text
                    #print('      + perform:', cmd.perform(self.__launch_context))
                elif isinstance(cmd, launch.actions.pop_launch_configurations.PopLaunchConfigurations):
                    print('      + CMD:', cmd.describe(), dir(cmd))
                elif isinstance(cmd, launch.substitutions.local_substitution.LocalSubstitution):
                    print('      + CMD Subst:',
                          cmd.expression, dir(cmd.expression))
                    # print('      + perform:', cmd.perform(self.__launch_context))
                else:
                    print('      + CMD OTHER:', cmd, dir(cmd))
                result += '_'
        return result

    def get_robot_icon(self):
        '''
        Returns the value of the `/robot_icon` parameter or None
        '''
        try:
            return self.roscfg.params['/robot_icon'].value
        except Exception:
            pass
        return None
