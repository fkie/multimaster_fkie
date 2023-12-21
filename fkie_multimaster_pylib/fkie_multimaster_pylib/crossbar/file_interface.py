# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
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

import json
import os


class RosPackage:
    def __init__(self, name: str, path: str) -> None:
        self.name = name
        self.path = path

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class PathItem:
    """
    :param str path: absolute path of the file or directory
    :param float mtime: time of last modification of path. The return value is a number giving the number of seconds since the epoch
    :param int size: size, in bytes, of path
    :param str path_type: one of types {file, dir, symlink, package}
    """

    def __init__(self, path: str, mtime: float, size: int, path_type: str) -> None:
        self.path = path
        self.mtime = mtime
        self.size = size
        self.type = path_type

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class LogPathItem:
    """
    :param str node: complete node name
    :param str screen_log: the absolute path to the screen log file.
    :param bool screen_log_exists: False if the file does not exists.
    :param str ros_log: the absolute path to the ros log file.
    :param bool ros_log_exists: False if the file does not exists.
    """

    def __init__(
        self,
        node: str,
        screen_log: str = "",
        screen_log_exists: bool = False,
        ros_log: str = "",
        ros_log_exists: bool = False,
    ) -> None:
        self.node = node
        self.screen_log = screen_log
        self.screen_log_exists = screen_log_exists
        self.ros_log = ros_log
        self.ros_log_exists = ros_log_exists

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)


class FileItem:
    """
    :param str path: absolute path of the file or directory
    :param float mtime: time of last modification of path. The return value is a number giving the number of seconds since the epoch
    :param int size: size, in bytes, of path
    :param str value: content of the file
    """

    def __init__(
        self,
        path: str,
        mtime: float = 0,
        size: int = 0,
        value: str = "",
        encoding="utf-8",
    ) -> None:
        self.path = path
        self.fileName = os.path.split(path)[-1]
        self.mtime = mtime
        self.size = size
        self.extension = path.rsplit(".", 1)[-1]
        self.value = value
        self.encoding = encoding

    def __str__(self):
        return json.dumps(dict(self), ensure_ascii=False)
