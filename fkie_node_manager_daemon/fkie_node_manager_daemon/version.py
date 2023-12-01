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

try:
    from fkie_node_manager_daemon import pkg_version
except ImportError:
    pass

VERSION = 'unknown'
DATE = 'unknown'


def detect_version(rosnode, package):
    '''
    Try to detect the current version from git, installed VERSION/DATE files or package.xml from file created while build.
    '''
    global VERSION
    global DATE
    if VERSION != 'unknown':
        return VERSION, DATE
    try:
        VERSION = pkg_version.version if (hasattr(pkg_version.version, "decode")) else pkg_version.version
        DATE = pkg_version.date if (hasattr(pkg_version.date, "decode")) else pkg_version.date
        rosnode.get_logger().info("detected version: %s (%s)" % (VERSION, DATE))
    except Exception:
        pass
    return VERSION, DATE
