# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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

import os
import re

try:
    from urlparse import urlparse
except ImportError:
    from urllib.parse import urlparse

import roslib.names
import rospy

from fkie_multimaster_msgs.defines import EMPTY_PATTERN
from fkie_multimaster_msgs.logging.logging import Log


IP4_PATTERN = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}")


def get_hostname(url):
    '''
    Extracts the hostname from given url.

    :param str url: the url to parse
    :return: the hostname or `None`, if the url is `None` or `invalid`
    :rtype: str
    :see: http://docs.python.org/library/urlparse.html
    '''
    if url is None:
        return None
    o = urlparse(url)
    hostname = o.hostname
    if hostname is None:
        div_idx = url.find(':')
        if div_idx > -1:
            hostname = url[0:div_idx]
        else:
            hostname = url
    return hostname


def resolve_url(interface_url, pwd='.'):
    '''
    The supported URL begins with `file:///`, `package://` or `pkg://`.
    The package URL will be resolved to a valid file path. If the file is in a
    subdirectory, you can replace the subdirectory by `///`.

    E.g.: `package://fkie_master_discovery///master_discovery.launch`

    :raise ValueError: on invalid URL or not existent file
    :return: the file path
    '''
    filename = ''
    if interface_url:
        if interface_url.startswith('file://'):
            filename = interface_url[7:]
        elif interface_url.startswith('package://') or interface_url.startswith('pkg://'):
            length = 6 if interface_url.startswith('pkg://') else 10
            pkg_name, _, pkg_path = interface_url[length:].partition('/')
            if pkg_path.startswith('//'):
                paths = roslib.packages.find_resource(
                    pkg_name, pkg_path.strip('/'))
                if len(paths) > 0:
                    # if more then one launch file is found, take the first one
                    filename = paths[0]
            else:
                pkg_dir = roslib.packages.get_pkg_dir(pkg_name)
                filename = os.path.join(pkg_dir, pkg_path)
        else:
            filename = interface_url
        if filename == '.':
            filename = ''
        if filename:
            filename = os.path.join(pwd, filename)
            if not os.path.exists(filename):
                raise ValueError(
                    'unsupported interface URL or interface file not found: ' + interface_url)
    return filename


def read_interface(interface_file):
    '''
    Reads the given file. You can use :mod:`fkie_master_discovery.common.resolve_url()`
    to resolve an URL to a file.

    :param str interface_file: the file containing the interface.
    :raise ValueError: on error while read interface
    :return: directory with content of the given file
    '''
    data = {}
    with open(interface_file, 'r') as f:
        iface = f.read()
        # parse Interface file / YAML text
        # - lazy import
        import yaml
        try:
            data = yaml.load(iface)
            if data is None:
                data = {}
        except yaml.MarkedYAMLError as e:
            if not interface_file:
                raise ValueError(
                    "Error within YAML block:\n\t%s\n\nYAML is:\n%s" % (str(e), iface))
            else:
                raise ValueError("file %s contains invalid YAML:\n%s" %
                                 (interface_file, str(e)))
        except Exception as e:
            if not interface_file:
                raise ValueError(
                    "invalid YAML: %s\n\nYAML is:\n%s" % (str(e), iface))
            else:
                raise ValueError("file %s contains invalid YAML:\n%s" %
                                 (interface_file, str(e)))
    return data


def create_pattern(param, data, has_interface, default=[], mastername=''):
    '''
    Create and compile the regular expression for given parameter. The data is
    taken from `data`. If the data was read from the interface file, then you have
    to set the `has_interface` to True. If `has_interface` is False, the data will
    be ignored and the parameter will be read from ROS parameter server.
    If resulting value is an empty list, `\\\\b` (http://docs.python.org/2/library/re.html)
    will be added to the pattern as `EMPTY_PATTERN`.

    :param str param: parameter name
    :param dict data: The dictionary, which can contain the parameter name and value.
                      The `data` will be ignored, if `has_interface` is `False`.
    :param bool has_interface: `True`, if valid data is available.
    :param list default: Default value will be added to the data
    :return: the compiled regular expression
    :rtype: The result of `re.compile()`
    '''
    def_list = default
    if has_interface:  # read the parameter from the sync interface data
        if param in data and data[param]:
            for item in data[param]:
                _parse_value(item, mastername, def_list)
    else:  # reads the patterns from the ROS parameter server
        rp = get_ros_param('~%s' % param, [])
        _parse_value(rp, mastername, def_list)
        # reads the mastername specific parameters
        if mastername:
            rph = get_ros_param(
                '~%s' % roslib.names.ns_join(mastername, param), [])
            if isinstance(rp, list):
                def_list[len(def_list):] = rph
            else:
                def_list.append(rph)
    def_list = list(set(def_list))
    return gen_pattern(def_list, param, print_info=True, mastername=mastername)


def get_ros_param(name, default):
    try:
        return rospy.get_param(name, default)
    except Exception:
        pass
    return default


def _parse_value(value, mastername, def_list):
    if isinstance(value, dict):
        # this are mastername specific remapings
        if mastername and mastername in value:
            if isinstance(value[mastername], list):
                def_list[len(def_list):] = value[mastername]
            else:
                def_list.append(value[mastername])
    elif isinstance(value, list):
        for item in value:
            if isinstance(item, dict):
                # this are mastername specific remapings
                if mastername and mastername in item:
                    if isinstance(item[mastername], list):
                        def_list[len(def_list):] = item[mastername]
                    else:
                        def_list.append(item[mastername])
            else:
                def_list.append(item)
    else:
        def_list.append(value)


def gen_pattern(filter_list, name, print_info=True, mastername=None):
    if print_info:
        if mastername is not None and mastername:
            Log.info(f'[{mastername}] {name}: {filter_list}')
        else:
            Log.info(f'{name}: {filter_list}')
    def_list = [''.join(['\A', n.strip().replace('*', '.*'), '\Z'])
                for n in filter_list]
    if def_list:
        return re.compile('|'.join(def_list), re.I)
    return EMPTY_PATTERN


def is_empty_pattern(re_object):
    '''
    Returns the value of `EMPTY_PATTERN`.
    '''
    return re_object == EMPTY_PATTERN
