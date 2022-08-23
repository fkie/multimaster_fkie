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


EFILE_CHANGED = 125
''':var EFILE_CHANGED: file changed in meantime.'''
EFILE_REMOVED = 126
''':var EFILE_REMOVED: file removed in meantime.'''


class FileItem(object):
    FILE = 0
    DIR = 1
    SYMLINK = 2
    PACKAGE = 3

    def __init__(self, path, path_type, size, mtime):
        self.path = path
        self.type = path_type
        self.size = size
        self.mtime = mtime
