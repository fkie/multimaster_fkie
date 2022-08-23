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

from typing import Text


class ListSelectionRequest(Exception):
    ''' '''

    def __init__(self, choices, error):
        Exception.__init__(self)
        self.choices = choices
        self.error = error

    def __repr__(self):
        return "%s <choices=%s>::%s" % (self.__class__, str(self.choices), repr(self.error))

    def __str__(self):
        return self.error


class BinarySelectionRequest(ListSelectionRequest):
    pass


class LaunchSelectionRequest(ListSelectionRequest):
    pass


class ParamSelectionRequest(ListSelectionRequest):
    pass


class StartException(Exception):
    pass


class AlreadyOpenException(Exception):

    def __init__(self, path, error):
        Exception.__init__(self)
        self.path = path
        self.error = error

    def __repr__(self):
        return "%s <path=%s>::%s" % (self.__class__, utf8(self.path), repr(self.error))

    def __str__(self):
        return self.error


class ResourceNotFound(AlreadyOpenException):
    pass


class RemoteException(Exception):

    def __init__(self, code, error):
        Exception.__init__(self)
        self.code = code
        self.error = error

    def __repr__(self):
        return "%s <code=%s>::%s" % (self.__class__, self.code, repr(self.error))

    def __str__(self):
        return self.error


class ConnectionException(Exception):

    def __init__(self, remote, error):
        Exception.__init__(self)
        self.remote = remote
        self.error = error

    def __repr__(self):
        return "%s %s::%s" % (self.__class__, self.remote, repr(self.error))

    def __str__(self):
        return self.error
