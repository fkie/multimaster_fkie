
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

from typing import Dict
from typing import List
from typing import Text
from typing import Tuple
from typing import Union

import os

try:
    from ament_index_python import get_resource
    from ament_index_python import get_packages_with_prefixes
    from ament_index_python.packages import get_package_share_directory
    AMENT_SUPPORTED = True
except ImportError:
    AMENT_SUPPORTED = False

try:
    from catkin_pkg.package import parse_package
    CATKIN_SUPPORTED = True
except ImportError:
    CATKIN_SUPPORTED = False

PACKAGE_FILE = 'package.xml'

_get_pkg_path_var = None


PACKAGE_CACHE = {}
SOURCE_PATH_TO_PACKAGES = {}


def get_packages(path: Union[Text, None]) -> Dict[Text, Text]:
    result = {}
    if path is None and AMENT_SUPPORTED:
        # we use ament to get the list of all packages
        return get_packages_with_prefixes()
    # TODO: this is old style code
    if os.path.isdir(path):
        fileList = os.listdir(path)
        if CATKIN_SUPPORTED and PACKAGE_FILE in fileList:
            try:
                pkg = parse_package(path)
                return {pkg.name: path}
            except Exception:
                pass
            return {}
        for f in fileList:
            ret = get_packages(os.path.join(path, f))
            result.update(ret)
    return result


def get_name(path: Text) -> Tuple[Text, Text]:
    '''
    The results are cached!

    :return: Returns for given directory a tuple of package name and package path.
    :rtype: tuple(str, str), empty strings if no package was found
    '''
    if path and path != os.path.sep:
        dir_path = path
        if not os.path.isdir(dir_path):
            dir_path = os.path.dirname(dir_path)
        if dir_path in PACKAGE_CACHE:
            return PACKAGE_CACHE[dir_path]
        try:
            fileList = os.listdir(dir_path)
            if CATKIN_SUPPORTED and PACKAGE_FILE in fileList:
                try:
                    pkg = parse_package(os.path.join(
                        dir_path, os.path.join(dir_path, PACKAGE_FILE)))
                    PACKAGE_CACHE[dir_path] = (pkg.name, dir_path)
                    return (pkg.name, dir_path)
                except Exception:
                    return ('', '')
            dir_path = os.path.dirname(dir_path)
            pname, pdir = get_name(dir_path)
            if pname:
                PACKAGE_CACHE[dir_path] = (pname, pdir)
                return (pname, pdir)
        except OSError:
            return ('', '')
    PACKAGE_CACHE[path] = ('', '')
    return ('', '')


def is_package(file_list: List[Text]) -> bool:
    return CATKIN_SUPPORTED and PACKAGE_FILE in file_list


def get_path(package_name: Text) -> Text:
    ''' :noindex: '''
    if AMENT_SUPPORTED:
        _, package_path = get_resource('packages', package_name)
        return package_path
    else:
        global _get_pkg_path_var
        if _get_pkg_path_var is None:
            try:
                import rospkg
                rp = rospkg.RosPack()
                _get_pkg_path_var = rp.get_path
            except ImportError:
                import roslib
                _get_pkg_path_var = roslib.packages.get_pkg_dir
        return _get_pkg_path_var(package_name)


def reset_cache() -> None:
    global _get_pkg_path_var
    _get_pkg_path_var = None
    global PACKAGE_CACHE
    PACKAGE_CACHE = {}
    global SOURCE_PATH_TO_PACKAGES
    SOURCE_PATH_TO_PACKAGES = {}

