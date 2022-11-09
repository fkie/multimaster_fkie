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


import glob
import os
import re

import asyncio
from autobahn import wamp
import json
import os
from typing import List
from fkie_multimaster_msgs import ros_pkg
from fkie_multimaster_msgs.crossbar.base_session import CrossbarBaseSession
from fkie_multimaster_msgs.crossbar.base_session import SelfEncoder
from fkie_multimaster_msgs.crossbar.file_interface import RosPackage
from fkie_multimaster_msgs.crossbar.file_interface import PathItem
from fkie_multimaster_msgs.crossbar.file_interface import LogPathItem
from fkie_multimaster_msgs.logging.logging import Log
from fkie_multimaster_msgs.system.screen import get_logfile
from fkie_multimaster_msgs.system.screen import get_ros_logfile


class FileServicer(CrossbarBaseSession):

    FILE_CHUNK_SIZE = 1024

    def __init__(self, loop: asyncio.AbstractEventLoop, realm: str = 'ros', port: int = 11911):
        Log.info("Create ROS2 file manager servicer")
        CrossbarBaseSession.__init__(self, loop, realm, port)
        # TODO: clear cache after detected change or time?
        self.CB_DIR_CACHE = {}

    def stop(self):
        '''
        '''
        self.shutdown()

    @wamp.register('ros.packages.get_list')
    def getPackageList(self, clear_cache: bool = False) -> List[RosPackage]:
        Log.info('Request to [ros.packages.get_list]')
        clear_cache = False
        if clear_cache:
            try:
                from roslaunch import substitution_args
                import rospkg
                substitution_args._rospack = rospkg.RosPack()
            except Exception as err:
                Log.warn(f"Cannot reset package cache: {err}")
        package_list: List[RosPackage] = []
        # fill the input fields
        ret = ros_pkg.get_packages(None)
        for name, path in ret.items():
            package = RosPackage(
                name=name, path=os.path.join(path, 'share', name))
            package_list.append(package)
        return json.dumps(package_list, cls=SelfEncoder)

    @wamp.register('ros.path.get_log_paths')
    def getLogPaths(self, nodes: List[str]) -> List[LogPathItem]:
        Log.info('Request to [ros.path.get_log_paths] for %s' % nodes)
        result = []
        for node in nodes:
            namespace = None
            node_name = node

            namespace_search = re.search('/(.*)/', node_name)
            if namespace_search is not None:
                namespace = f'/{namespace_search.group(1)}'
                node_name = node.replace(f'/{namespace}/', '')

            screen_log = get_logfile(
                node=node_name, for_new_screen=True, namespace=namespace)
            ros_log = get_ros_logfile(node)
            log_path_item = LogPathItem(node,
                                        screen_log=screen_log,
                                        screen_log_exists=os.path.exists(
                                            screen_log),
                                        ros_log=ros_log,
                                        ros_log_exists=os.path.exists(ros_log))
            result.append(log_path_item)
        return json.dumps(result, cls=SelfEncoder)

    @wamp.register('ros.path.get_list')
    def getPathList(self, inputPath: str) -> List[PathItem]:
        Log.info('Request to [ros.path.get_list] for %s' % inputPath)
        path_list: List[PathItem] = []
        # list the path
        dirlist = os.listdir(inputPath)
        for cfile in dirlist:
            path = os.path.normpath('%s%s%s' % (inputPath, os.path.sep, cfile))
            if os.path.isfile(path):
                path_list.append(PathItem(path=path, mtime=os.path.getmtime(
                    path), size=os.path.getsize(path), path_type='file'))
            elif path in self.CB_DIR_CACHE:
                path_list.append(PathItem(path=path, mtime=os.path.getmtime(
                    path), size=os.path.getsize(path), path_type=self.CB_DIR_CACHE[path]))
            elif os.path.isdir(path):
                try:
                    fileList = os.listdir(path)
                    file_type = None
                    if ros_pkg.is_package(fileList):
                        file_type = 'package'
                    else:
                        file_type = 'dir'
                    self.CB_DIR_CACHE[path] = file_type
                    path_list.append(PathItem(path=path, mtime=os.path.getmtime(
                        path), size=os.path.getsize(path), path_type=file_type))
                except Exception as _:
                    pass
        return json.dumps(path_list, cls=SelfEncoder)

    @wamp.register('ros.path.get_list_recursive')
    def getPathListRecursive(self, inputPath: str) -> List[PathItem]:
        Log.info(
            'Request to [ros.path.get_list_recursive] for %s' % inputPath)
        path_list: List[PathItem] = []

        for filename in glob.iglob(inputPath + '**/**', recursive=True):
            if filename == inputPath:
                continue

            if os.path.isfile(filename):
                path_list.append(PathItem(path=filename, mtime=os.path.getmtime(
                    filename), size=os.path.getsize(filename), path_type='file'))

        return json.dumps(path_list, cls=SelfEncoder)
