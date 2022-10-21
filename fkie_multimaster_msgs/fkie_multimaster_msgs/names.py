
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

from typing import Text

from fkie_multimaster_msgs.defines import PRIV_NAME
from fkie_multimaster_msgs.defines import SEP


def ns_join(ns: Text, name: Text) -> Text:
    """
    Join a namespace and name. If name is unjoinable (i.e. ~private or
    /global) it will be returned without joining

    :param str ns: namespace ('/' and '~' are both legal). If ns is the empty string, name will be returned.
    :param str name: a legal name
    :return: name concatenated to ns, or name if it is unjoinable.
    :rtype: str
    """
    if not name:
        return name
    if name[0] == SEP or name[0] == PRIV_NAME:
        return name
    if ns == PRIV_NAME:
        return PRIV_NAME + name
    if not ns:
        return name
    if ns[-1] == SEP:
        return ns + name
    return ns + SEP + name
