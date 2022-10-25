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

import os
from typing import Tuple
from typing import Union
from urllib.parse import urlparse


def equal_uri(url1: str, url2: str) -> bool:
    '''
    Removes to string after remove last slash character.
    '''
    return url1.rstrip(os.path.sep) == url2.rstrip(os.path.sep)


def get_port(url: str) -> Union[int, None]:
    '''
    Extracts the port from given url.

    :param str url: the url to parse
    :return: the port or `None`, if the url is `None` or `invalid`
    :rtype: int
    :see: http://docs.python.org/library/urlparse.html
    '''
    if url is None:
        return None
    if not url:
        return url
    result = None
    try:
        o = urlparse(url)
        result = o.port
        if result is None:
            res = url.split(':')
            if len(res) == 2:
                result = int(res[1])
    finally:
        return result


def split_uri(uri: str) -> Union[Tuple[str, str, int], None]:
    '''
    Splits URI or address into scheme, address and port and returns them as tuple.
    Scheme or tuple are empty if no provided.
    :param str uri: some URI or address
    :rtype: (str, str, int)
    '''
    (scheme, hostname, port) = ('', '', -1)
    if uri is None:
        return None
    if not uri:
        return uri
    try:
        o = urlparse(uri)
        scheme = o.scheme
        hostname = o.hostname
        port = o.port
    except AttributeError:
        pass
    if hostname is None:
        res = uri.split(':')
        if len(res) == 2:
            hostname = res[0]
            port = res[1]
        elif len(res) == 3:
            if res[0] == 'SHM':
                hostname = 'localhost'
                port = res[2]
            else:
                # split if more than one address
                hostname = res[1].strip('[]')
                port = res[2]
        elif len(res) == 4 and res[1] == 'SHM':
            hostname = 'localhost'
            port = res[3]
        else:
            hostname = uri
            port = -1
    try:
        port = int(port)
    except TypeError:
        port = -1
    return (scheme, hostname, port)
