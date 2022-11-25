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

from typing import Dict
from typing import Iterable
from typing import List
from numbers import Number
from typing import Optional
from typing import Set
from typing import Text
from typing import Tuple
from typing import Union

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
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch.utilities import normalize_to_list_of_substitutions
from launch.launch_description import LaunchDescription
import launch_ros

from fkie_multimaster_msgs.crossbar.launch_interface import LaunchArgument
from fkie_multimaster_msgs.crossbar.launch_interface import LaunchNodeInfo
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs import names
from fkie_multimaster_msgs import ros_pkg
from fkie_multimaster_msgs.defines import SEP

from launch.launch_context import LaunchContext


class LaunchConfigException(Exception):
    pass


class LaunchNodeWrapper(LaunchNodeInfo):

    _unique_names: Set[str] = set()
    _remapped_names: Dict[str, Set[str]] = {}

    def __init__(self, entity: launch.actions.ExecuteProcess, launch_description: Union[launch.LaunchDescription, launch.actions.IncludeLaunchDescription], launch_context: launch.LaunchContext) -> None:
        self._entity = entity
        self._launch_description = launch_description
        self._launch_context = launch_context
        print("launch_context:", launch_context.locals,
              dir(launch_context.locals))
        print("launch_context, current_launch_file_path,:",
              getattr(launch_context.locals, "current_launch_file_path", 'NONE'))
        if isinstance(self._entity, launch_ros.actions.Node):
            # Prepare the ros_specific_arguments list and add it to the context so that the
            # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
            ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
            if self._entity._Node__node_name is not None:
                ros_specific_arguments['name'] = '__node:={}'.format(
                    self._entity._Node__expanded_node_name)
            if self._entity._Node__expanded_node_namespace != '':
                ros_specific_arguments['ns'] = '__ns:={}'.format(
                    self._entity._Node__expanded_node_namespace)

            # Give extensions a chance to prepare for execution
            for extension in self._entity._Node__extensions.values():
                cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
                    self._launch_context,
                    ros_specific_arguments,
                    self
                )
                self._entity._Node.cmd.extend(cmd_extension)

            self._launch_context.extend_locals(
                {'ros_specific_arguments': ros_specific_arguments})

        node_name, unique_name = self._get_name()
        LaunchNodeInfo.__init__(
            self, unique_name=unique_name, node_name=node_name)
        self.node_namespace = self._get_namespace()
        self.package = self._get_node_package()
        self.executable = self._get_node_executable()
        self.respawn = self._get_respawn()
        self.respawn_delay = self._get_respawn_delay()
        if isinstance(self._launch_description, launch.actions.IncludeLaunchDescription):
            self.file_name = self._launch_description._get_launch_file()
        else:
            self.launch_name = getattr(
                self._launch_description, 'launch_name', '')
            self.file_name = self.launch_name
        self.composable_container: str = ''
        self.launch_prefix = self._get_launch_prefix()
        self.parameters = self._get_parameter_arguments()
        self.args = self._get_arguments()
        self.cmd = self._to_string(self._entity.cmd)
        self.cwd = self._to_string(self._entity.cwd)
        self.env = self._to_tuple_list(self._entity.env)
        self.additional_env = self._to_tuple_list(self._entity.additional_env)
        self.launch_prefix = self._to_string(self._entity.prefix)

        #  remap_args: List[Tuple[str, str]] = None,
        #  output: str = '',
        #  output_format: str = '',
        #  sigterm_timeout: str = '',
        #  sigkill_timeout: str = '',
        #  on_exit: List[Any] = [],
        #  required: bool = False,
        #  file_name: str = '',
        #  file_range: Dict[str, Number] = {"startLineNumber": 0,
        #                                   "endLineNumber": 0,
        #                                   "startColumn": 0,
        #                                   "endColumn": 0},
        #  launch_context_arg: str = '',
        #  launch_name: str = ''
        #  composable_container: str = ''

    def __del__(self):
        try:
            LaunchNodeWrapper._unique_names.remove(self.unique_name)
            Log.debug(f"removed from unique {self.unique_name}")
        except (ValueError, KeyError):
            # remove index
            LaunchNodeWrapper._remapped_names[self.node_name].remove(
                self.unique_name)
            Log.debug(f"removed from remapped {self.unique_name}")

    def _get_node_executable(self):
        if getattr(self, 'executable', ''):
            return self.executable
        result = ''
        # no name was set for Node or ExecuteProcess => use executable
        if not result:
            result = getattr(self._entity, '_Node__executable', '')
        # no name was set for Node or ExecuteProcess => use node_executable; before foxy
        if not result:
            result = getattr(self._entity, '_Node__node_executable', '')
        if result:
            if not isinstance(result, str):
                result = launch.utilities.perform_substitutions(
                    self._launch_context, result)
        self.executable = result
        return result

    def _get_launch_prefix(self) -> str:
        return launch.utilities.perform_substitutions(
            self._launch_context, self._entity.prefix)

    def _get_respawn(self) -> bool:
        return self._entity._ExecuteProcess__respawn

    def _get_respawn_delay(self) -> Union[float, None]:
        return self._entity._ExecuteProcess__respawn_delay

    def _get_parameter_arguments(self):
        return getattr(self._entity, '_Node__expanded_parameter_arguments', [])

    def _get_arguments(self):
        return getattr(self._entity, '_Nodes__arguments', [])

    def _get_node_package(self) -> str:
        """Getter for node_package."""
        result = getattr(self._entity, '_Node__package', '')
        return result

    def _get_namespace(self) -> str:
        result = getattr(self._entity, 'expanded_node_namespace', SEP)
        if result == launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE:
            result = SEP
        return result

    def _get_name(self) -> Tuple[str, str]:
        result = ''
        # first get name from launch.ExecuteProcess
        result = getattr(self._entity, 'name', '')
        # get name from launch_ros.actions.Node
        if not result:
            result = getattr(self._entity, 'node_name', '')
        if result:
            if not isinstance(result, str):
                result = launch.utilities.perform_substitutions(
                    self._launch_context, result)
            if result.endswith(launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAME):
                result = ''
        if not result:
            # use executable as name
            result = self._get_node_executable()
        # try to create the name from command line
        if not result:
            result = self._get_name_from_cmd()
            if result:
                Log.info(f"Nodename '{result}' from cmd")
        # check for valid namespace
        if result and not result.startswith(SEP):
            ns = self._get_namespace()
            if ns != SEP:
                result = names.ns_join(ns, result)
        # if only the name is set in the launch file. 'node_name' returns name with unspecified namespace
        result = result.replace(
            f"{launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE}/", '')
        if not result:
            Log.warn("No name for node found: %s %s" %
                     (type(self._entity), dir(self._entity)))
        # check for unique name
        unique_name = result
        if result in LaunchNodeWrapper._unique_names:
            # the name already exists! create a unique one
            name_set = set()
            if result in LaunchNodeWrapper._remapped_names:
                name_set = LaunchNodeWrapper._remapped_names[result]
            else:
                LaunchNodeWrapper._remapped_names[result] = name_set
            index = 2
            unique_name = f"{result}_{index}"
            while unique_name in name_set:
                index += 1
                unique_name = f"{result}_{index}"
            name_set.add(unique_name)
        else:
            LaunchNodeWrapper._unique_names.add(result)
        Log.info(f"create node wrapper with name '{result}'")
        return (result, unique_name)

    def _get_name_from_cmd(self):
        result = ''
        cmd_list = getattr(self._entity, 'cmd', [])
        if cmd_list:
            result = launch.utilities.perform_substitutions(
                self._launch_context, cmd_list[0])
        result = os.path.basename(result.replace(' ', '_'))
        return result

    def _to_string(self, value: Union[List[List], List[launch.Substitution], str, None]) -> Union[str, None]:
        result = ''
        if isinstance(value, str):
            result = value
        elif value and isinstance(value[0], launch.Substitution):
            result += launch.utilities.perform_substitutions(
                self._launch_context, value)
        elif isinstance(value, List):
            result += ' '.join([self._to_string(val) for val in value])
        elif value is not None:
            Log.warn("IGNORED while _to_string", value)
        else:
            result = None
        return result

    def _to_tuple_list(self, value: Union[List[Tuple[List[launch.Substitution], List[launch.Substitution]]], None]) -> Union[List[Tuple[str, str]], None]:
        result = []
        if value is not None:
            for val1, val2 in value:
                result.append((launch.utilities.perform_substitutions(
                    self._launch_context, val1), launch.utilities.perform_substitutions(
                    self._launch_context, val2)))
        else:
            result = None
        return result


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
        self.__package = ros_pkg.get_name(os.path.dirname(self.__launchfile))[
            0] if package is None else package
        self._nodes: List[LaunchNodeWrapper] = []
        self.__nmduri = daemonuri
        self.provided_launch_arguments = launch_arguments
        self.launch_arguments: List[LaunchArgument] = []
        argv = sys.argv[1:]
        argv.extend(["%s:=%s" % (name, value)
                     for (name, value) in launch_arguments])
        self.__launch_context = context
        if context is None:
            self.__launch_context = LaunchContext(argv=argv)
        for (name, value) in launch_arguments:
            self.__launch_context.launch_configurations[name] = value
        self.__launch_description = get_launch_description_from_any_launch_file(
            self.filename)
        # self.__launch_description.visit(self.__launch_context)
        #self.ldsource = launch.LaunchDescriptionSource(launch_description=self.__launch_description, location=launch_file)
        #self.__launch_description = self.ldsource.get_launch_description(self.__launch_context)
        #ild = IncludeLaunchDescription(launch_description_source=self.ldsource, launch_arguments=launch_arguments)
        #rr = ild.execute(self.__launch_context)
        # for key in rr:
        #    print("r", key, type(key))
        #    if isinstance(key, launch.launch_description.LaunchDescription):
        #        print("apply ld")
        #        self.__launch_description = key
#        for key, val in self.__launch_context.launch_configurations.items():
#            print("KK", key, val)

        #print("LD", dir(self.__launch_description))
        # print("frontend_parsers", launch.frontend.parser.Parser.frontend_parsers)
        # launch.frontend.parser.Parser.load_launch_extensions()
        # launch.frontend.parser.Parser.load_parser_implementations()
        # print("frontend_parsers", launch.frontend.parser.Parser.frontend_parsers)
        # entity, parser = launch.frontend.parser.Parser.load(self.filename)
        # print("DIFFLOAD", entity, parser)
        self._included_files: List[IncludeLaunchDescription] = []
        self.__launch_description.launch_name = self.filename
        self._load(current_file=self.filename)
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

    def __del__(self):
        Log.info(f"delete Launch config {self.filename}")
        self._nodes.clear()

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

    def _unload(self):
        print("UNLOAD")
        self.launch_arguments.clear()
        self._included_files.clear()
        self._nodes.clear()

    def _load(self, sub_obj=None, launch_description=None, current_file: str = '', ident: str = '') -> None:
        current_launch_description = launch_description
        if sub_obj is None:
            sub_obj = self.__launch_description
        if current_launch_description is None:
            current_launch_description = self.__launch_description

        # import traceback
        # print(traceback.format_stack())
        # print("Launch arguments:")
        # for la in self.__launch_description.get_launch_arguments():
        #     print(la.name, launch.utilities.perform_substitutions(self.context, la.default_value))

        entities = None
        if hasattr(sub_obj, 'get_sub_entities'):
            print(ident, "GET SUB ENTITY")
            entities = getattr(sub_obj, 'get_sub_entities')()
        elif hasattr(sub_obj, 'entities'):
            print(ident, "GET ENTITY")
            entities = getattr(sub_obj, 'entities')
        if entities is not None:
            for entity in entities:
                # if isinstance(entity, launch.launch_description.LaunchDescription):
                #     current_launch_description = entity
                print(ident, f"typeID: {type(entity)} {entity}")
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
                    print("IIII", entity._Node__node_executable)

                    self._nodes.append(LaunchNodeWrapper(
                        entity, current_launch_description, self.context))
                    # if entity.expanded_node_namespace == launch_ros.actions.node.Node.UNSPECIFIED_NODE_NAMESPACE:
                    #     entity.__expanded_node_namespace = ''
#                    if entity.condition is not None:
#                        print("CONDITION: ",
#                              entity.condition.evaluate(self.context))
                elif isinstance(entity, launch.actions.execute_process.ExecuteProcess):
                    self._nodes.append(LaunchNodeWrapper(
                        entity, current_launch_description, self.__launch_context))
#                    print("EXEC", type(entity), dir(entity))
                    # entity._perform_substitutions(self.context)
                elif isinstance(entity, launch.actions.declare_launch_argument.DeclareLaunchArgument):
                    #                    print('  perform ARG:', entity.name, launch.utilities.perform_substitutions(
                    #                        self.context, entity.default_value))
                    cfg_actions = entity.execute(self.__launch_context)
                    if cfg_actions is not None:
                        for cac in cfg_actions:
                            print(ident, '->', type(cac), cac)
#                    print('  perform ARG after execute:', entity.name, launch.utilities.perform_substitutions(
#                        self.context, entity.default_value))
                    self._load(entity, current_launch_description,
                               current_file=current_file, ident=ident+'  ')
                    if current_file:
                        self.context.extend_locals({
                            'current_launch_file_path': current_file,
                        })
                elif isinstance(entity, launch.actions.include_launch_description.IncludeLaunchDescription):
                    # launch.actions.declare_launch_argument.DeclareLaunchArgument
                    try:
                        cfg_actions = entity.execute(self.__launch_context)
                        if cfg_actions is not None:
                            for cac in cfg_actions:
                                print(ident, '>>', type(cac), cac)
                            ild = entity.launch_description_source.get_launch_description(
                                self.context)
                            for cac in cfg_actions[:-1]:
                                print(ident, '++', type(cac), cac)
                                ild.add_action(cac)
                        #current_launch_description = entity
                        self._included_files.append(entity)

#                        print("IncludeLaunchDescription", dir(), entity)
#                        for a in dir(entity):
                        # if not a.startswith('_')
#                            print(f"  {a}: {getattr(entity, a)}")
 #                       print("_get_launch_file", entity._get_launch_file())
  #                      print("_get_launch_file_directory",
   #                           entity._get_launch_file_directory())
   #                     print("get_launch_arguments - ",
   #                           entity.launch_arguments)
    #                    print("launch_configurations - ",
     #                         self.__launch_context.launch_configurations)
#                        for (name, value) in entity.launch_arguments:
#                            self.__launch_context.launch_configurations[name] = value
      #                      print(name, value)
                        self._load(
                            entity, entity, current_file=entity._get_launch_file(), ident=ident+'  ')
                        if current_file:
                            self.context.extend_locals({
                                'current_launch_file_path': current_file,
                            })
                    except launch.invalid_launch_file_error.InvalidLaunchFileError as err:
                      #                  print('err', dir(err))
                       #                 print('launch_description_source', dir(
                       #                     entity.launch_description_source.location))
                        raise Exception('%s (%s)' % (
                            err, entity.launch_description_source.location))
                elif hasattr(entity, 'execute'):
                   #             print("TY2", type(entity), dir(entity))
                    entity.execute(self.__launch_context)
        #        else:
        #            print("unknown entity:", entity, dir(entity))
        #            for a in dir(entity):
                    # if not a.startswith('_')
        #                print(f"  {a}: {getattr(entity, a)}")
                else:
                    print("unknown entity:", entity)
                    self._load(entity, current_launch_description,
                               current_file=current_file, ident=ident+'  ')
                    if current_file:
                        self.context.extend_locals({
                            'current_launch_file_path': current_file,
                        })
                if len(ident) > 10:
                    raise
#        print("get_launch_arguments", [launch.utilities.perform_substitutions(
#            self.context, l.default_value) for l in self.__launch_description.get_launch_arguments()])

    def nodes(self) -> List[LaunchNodeWrapper]:
        return self._nodes

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
    def get_launch_arguments(cls, filename: str, provided_args: list) -> List[LaunchArgument]:
        '''
        :param list(fkie_multimaster_msgs.crossbar.runtime_interface.RosParameter) provided_args: provided args used to set 'value' in returned args
        :return: a list with args being used in the roslaunch file.
        :rtype: list(kie_multimaster_msgs.crossbar.runtime_interface.RosParameter)
        '''
        context = LaunchContext()
        launch_description = get_launch_description_from_any_launch_file(
            filename)
        launch_arguments: List[launch.actions.declare_launch_argument.DeclareLaunchArgument] = launch_description.get_launch_arguments()
        result = []
        for argument_action in launch_arguments:
            value = ''
            for parg in provided_args:
                if argument_action.name == parg.name:
                    value = parg.value
                    break

            default_value = None
            if argument_action.default_value is not None:
                default_value = launch.utilities.perform_substitutions(
                    context, argument_action.default_value)
            arg = LaunchArgument(name=argument_action.name,
                                 value=value,
                                 default_value=default_value,
                                 description=argument_action.description,
                                 choices=argument_action.choices)
            result.append(arg)
        return result

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

    def get_node(self, name: str, daemonuri: str = '') -> Union[LaunchNodeWrapper, None]:
        '''
        Returns a configuration node for a given node name.

        :param str name: the name of the node.
        :return: the configuration node stored in this configuration
        :rtype: :class:`launch_ros.actions.node.Node` or None
        '''
        for item in self.nodes():
            if (item.unique_name == name):
                return item
        Log.warn("Node '%s' NOT found" % name)
        return None
