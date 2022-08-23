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
import platform
import socket
import sys
from urllib.parse import urlparse
from fkie_node_manager_daemon.host import get_host_name


NMD_DEFAULT_PORT = 11911
''':var NMD_DEFAULT_PORT: default port of node manager daemon.'''


def equal_uri(url1, url2):
    '''
    Removes to string after remove last slash character.
    '''
    return url1.rstrip(os.path.sep) == url2.rstrip(os.path.sep)


def nmduri(uri='', prefix='grpc://'):
    '''
    Determine for given url a gRPC-URI with `grpc://` scheme. If the
    given URI is a ROS-Master URI the method calculate new p
    ort by adding
    `NMD_SERVER_PORT_OFFSET`. If the given URI is empty we try to determine
    the ROS-Master URI from environment or from ROS-Master.

    :param str uri: empty or ROS-Master uri
    :param str prefix: the scheme can be replaced
    :return: URI with `grpc`-scheme.
    :rtype: str
    :raise ValueError: if uri is not empty and contains no scheme ('http', 'grpc')
    '''
    muri = uri
    if not muri:
        muri = 'grpc://%s:%d' % (get_host_name(), nmdport())
    o = urlparse(muri)
    port = o.port
    if o.scheme not in ['grpc']:
        raise ValueError("uri parameter does not contain a scheme of ['grpc']: %s" % uri)
    hostname = o.hostname
    if hostname in ['::']:
        hostname = get_host_name()
    return "%s%s:%d" % (prefix, hostname, port)


def port_from_uri(uri):
    o = urlparse(uri)
    return o.port


def nmdport():
    '''
    Returns the port for GRPC-server.
    The ROS_DOMAIN_ID increases the default port.
    
    :rtype: int
    '''
    port = NMD_DEFAULT_PORT
    if 'ROS_DOMAIN_ID' in os.environ:
        port = port + int(os.environ['ROS_DOMAIN_ID'])
    return port


def nmduri_from_path(grpc_path):
    '''
    Splits the gRPC-URI with scheme into URI and file path.

    :param str grpc_path: gRPC-URI with file path.
    :return: gRPC_URI without file path
    :rtype: str
    :raise ValueError: if grpc_path is empty or does not start with `grpc://`
    '''
    url, _path = split(grpc_path, with_scheme=True)
    return url


def join(uri, path):
    '''
    Creates gRPC-URI with file path from given URI and path.
    If given URI is ROS-Master URI it will be converted to gRPC-URI by :meth:`nmduri`

    :param str uri: URI of the node manager daemon
    :param str path: file path
    :return: gRPC-path
    :rtype: str
    '''
    if not path.startswith('grpc://'):
        if not uri.startswith('grpc://'):
            if path.startswith(os.path.sep) or not path:
                return "%s%s" % (nmduri(uri), path)
            return "%s%s%s" % (nmduri(uri), os.path.sep, path)
        elif path.startswith(os.path.sep) or not path:
            return '%s%s' % (uri, path)
        return '%s%s%s' % (uri, os.path.sep, path)
    return path


def split(grpc_path, with_scheme=False):
    '''
    Splits the gRPC-URI with scheme into URI and file path.

    :param str grpc_path: gRPC-URI with file path.
    :param bool with_scheme: if True the gRPC-URI contains also the `grpc://` scheme.
    :return: a tuple of gRPC_URI without file path and path
    :rtype: (str, str)
    :raise ValueError: if grpc_path is empty or does not start with `grpc://`
    '''
    url = grpc_path
    if not grpc_path:
        url = nmduri()
    if url and not url.startswith('grpc://'):
        raise ValueError("Invalid grpc path to split: %s; `grpc` scheme missed!" % grpc_path)
    url_parse_result = urlparse(url)
    if with_scheme:
        return ("%s://%s" % (url_parse_result.scheme, url_parse_result.netloc), url_parse_result.path)
    return (url_parse_result.netloc, url_parse_result.path)
