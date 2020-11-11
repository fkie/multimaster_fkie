# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Fraunhofer FKIE/CMS, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



from xml.dom.minidom import parse  # , parseString
import os
import re
import time

import roslaunch
import roslib
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from fkie_master_discovery.common import masteruri_from_master
from .common import package_name, utf8


class LaunchConfigException(Exception):
    pass


class LaunchConfig(object):
    '''
    A class to handle the ROS configuration stored in launch file.
    '''

    def __init__(self, launch_file, package=None, masteruri='', host='', argv=None, monitor_servicer=None):
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
        :param str masteruri: The URL of the ROS master.
        :param argv: the list the arguments needed for loading the given launch file
        :type argv: list(str)
        :raise roslaunch.XmlParseException: if the launch file can't be found.
        '''
        self._monitor_servicer = monitor_servicer
        self.__launchfile = launch_file
        self.__package = package_name(os.path.dirname(self.__launchfile))[0] if package is None else package
        self.__masteruri = masteruri if masteruri else masteruri_from_master(True)
        self.__roscfg = None
        self.argv = argv
        if self.argv is None:
            self.argv = []
        self.__reqTested = False
        self.__argv_values = dict()
        self.global_param_done = []  # masteruri's where the global parameters are registered
        self.__launch_id = '%.9f' % time.time()
        self._robot_description = None
        self._capabilities = None
        self.host = host if host else None
        self.resolve_dict = {}
        self.changed = True

#     def __del__(self):
#         pass

    @property
    def masteruri(self):
        '''
        :return: Returns the master URI (host) where the node of this config will be started.
        :rtype: str
        '''
        return self.__masteruri

    @property
    def roscfg(self):
        '''
        Holds a loaded launch configuration. It raises a LaunchConfigException on load error.

        :return: a previously loaded ROS configuration
        :rtype: :meth:`roslaunch.ROSLaunchConfig` <http://docs.ros.org/kinetic/api/roslaunch/html/> or None
        :any: :meth:`load`
        '''
        if self.__roscfg is not None:
            return self.__roscfg
        else:
            result, _ = self.load(self.argv)  # _:=argv
            if not result:
                raise LaunchConfigException("not all argv are setted properly!")
            return self.__roscfg

    @property
    def filename(self):
        '''
        Returns an existing path with file name or an empty string.

        :rtype: str
        '''
        if os.path.isfile(self.__launchfile):
            return self.__launchfile
        elif self.__package is not None:
            try:
                return roslib.packages.find_resource(self.packagename, self.launchname).pop()
            except Exception:
                raise LaunchConfigException('launch file %s not found!' % self.launchname)
        raise LaunchConfigException('launch file %s not found!' % self.__launchfile)

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
            roscfg = roslaunch.ROSLaunchConfig()
            loader = roslaunch.XmlLoader()
            self.argv = self.resolve_args(argv)
            loader.ignore_unset_args = False
            loader.load(self.filename, roscfg, verbose=False, argv=self.argv)
            self.__roscfg = roscfg
            if 'arg' in loader.root_context.resolve_dict:
                self.resolve_dict = loader.root_context.resolve_dict['arg']
            self.changed = True
            # check for depricated parameter
            diag_dep = DiagnosticArray()
            if self._monitor_servicer is not None:
                diag_dep.header.stamp = rospy.Time.now()
            for n in roscfg.nodes:
                node_fullname = roslib.names.ns_join(n.namespace, n.name)
                associations_param = roslib.names.ns_join(node_fullname, 'associations')
                if associations_param in roscfg.params:
                    ds = DiagnosticStatus()
                    ds.level = DiagnosticStatus.WARN
                    ds.name = node_fullname
                    ds.message = 'Deprecated parameter detected'
                    ds.values.append(KeyValue('deprecated', 'associations'))
                    ds.values.append(KeyValue('new', 'nm/associations'))
                    rospy.logwarn("'associations' is deprecated, use 'nm/associations'! found for node: %s in %s" % (node_fullname, self.filename))
                    diag_dep.status.append(ds)
            if self._monitor_servicer is not None:
                # set diagnostics
                self._monitor_servicer._monitor._callback_diagnostics(diag_dep)
        except roslaunch.XmlParseException as e:
            test = list(re.finditer(r"environment variable '\w+' is not set", utf8(e)))
            message = utf8(e)
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
                    raise LaunchConfigException("Can't resolve the argument `%s` argument: the argument referenced to itself!" % arg_name)
                if rec_inc > 100:
                    raise LaunchConfigException("Can't resolve the argument `%s` in `%s` argument: recursion depth of 100 reached!" % (arg_name, arg))
                if arg_name in argv_defaults:
                    argv_defaults[arg] = value.replace(value[arg_match.start():endIndex + 1], argv_defaults[arg_name])
                elif arg_name in argv_values:
                    argv_defaults[arg] = value.replace(value[arg_match.start():endIndex + 1], argv_values[arg_name])
                else:
                    raise LaunchConfigException("Can't resolve the argument `%s` in `%s` argument" % (arg_name, arg))
            else:
                raise LaunchConfigException("Can't resolve the argument in `%s` argument: `)` not found" % arg)
            value = argv_defaults[arg]
            arg_match = re.search(r"\$\(\s*arg\s*", value)

    def get_args(self):
        '''
        :return: a list with args being used in the roslaunch file. Only arg tags that are a direct child of <launch> will
                 be returned
        :rtype: list(str)
        :raise roslaunch.XmlParseException: on parse errors
        '''
        self._argv_values = dict()
        arg_subs = []
        args = []
        # get only the args in the top launch file
        for filename in [self.filename]:
            try:
                if filename.endswith('.launch') or filename.find('.launch.') > 0:
                    args[len(args):-1] = parse(filename).getElementsByTagName('arg')
            except Exception as e:
                raise roslaunch.XmlParseException("Invalid roslaunch XML syntax: %s" % e)
            for arg in args:
                arg_name = arg.getAttribute("name")
                if not arg_name:
                    raise roslaunch.XmlParseException("arg tag needs a name, xml is %s" % arg.toxml())
                # we only want argsargs at top level:
                if not arg.parentNode.tagName == "launch":
                    continue
                arg_default = arg.getAttribute("default")
                arg_value = arg.getAttribute("value")
                arg_sub = ''.join([arg_name, ':=', arg_default])
                if (not arg_value) and arg_sub not in arg_subs:
                    arg_subs.append(arg_sub)
                elif arg_value:
                    self.__argv_values[arg_name] = arg_value
        return arg_subs

    def _decode(self, val):
        '''
        Replaces the '\\n' by LF (Line Feed) and decode the string entry to unicode.

        :param str val: the string coding as system default
        :return: the decoded string
        :rtype: unicode or original on error
        '''
        result = val.replace("\\n ", "\n")
        try:
            result = utf8(result)
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
                            rospy.logwarn("WRONG format, expected: ['host', 'type', 'name', 'images', 'description'] -> ignore; param: %s" % param)
                        else:
                            for entry in p.value:
                                self._robot_description[entry[0]] = {'type': entry[1], 'name': entry[2], 'images': entry[3].split(','), 'description': self._decode(entry[4])}
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
                            rospy.logwarn("WRONG format, expected: ['name', 'type', 'images', 'description'] -> ignore; param: %s" % param)
                        else:
                            for entry in p.value:
                                capabilies_descr[entry[0]] = {'type': '%s' % entry[1], 'images': entry[2].split(','), 'description': self._decode(entry[3])}
            # get the capability nodes
            for item in self.roscfg.nodes:
                node_fullname = roslib.names.ns_join(item.namespace, item.name)
                machine_name = item.machine_name if item.machine_name is not None and not item.machine_name == 'localhost' else ''
                added = False
                cap_param = roslib.names.ns_join(node_fullname, 'capability_group')
                cap_ns = node_fullname
                # find the capability group parameter in namespace
                while cap_param not in self.roscfg.params and cap_param.count(roslib.names.SEP) > 1:
                    cap_ns = roslib.names.namespace(cap_ns).rstrip(roslib.names.SEP)
                    if not cap_ns:
                        cap_ns = roslib.names.SEP
                    cap_param = roslib.names.ns_join(cap_ns, 'capability_group')
                if cap_ns == node_fullname:
                    cap_ns = item.namespace.rstrip(roslib.names.SEP)
                    if not cap_ns:
                        cap_ns = roslib.names.SEP
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
                        result[machine_name][ns][p.value]['nodes'].append(node_fullname)
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

    def get_node(self, name, masteruri='', host=''):
        '''
        Returns a configuration node for a given node name.

        :param str name: the name of the node.
        :return: the configuration node stored in this configuration
        :rtype: :class:`roslaunch.Node` <http://docs.ros.org/kinetic/api/roslaunch/html/> or None
        '''
        nodename = os.path.basename(name)
        namespace = os.path.dirname(name).strip(roslib.names.SEP)
        for item in self.roscfg.nodes:
            if (item.name == nodename) and (item.namespace.strip(roslib.names.SEP) == namespace):
                return item
        return None

    def get_robot_icon(self):
        '''
        Returns the value of the `/robot_icon` parameter or None
        '''
        try:
            return self.roscfg.params['/robot_icon'].value
        except Exception:
            pass
        return None
