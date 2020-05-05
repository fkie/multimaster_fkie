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



import os
try:
    from urlparse import urlparse  # python 2 compatibility
except ImportError:
    from urllib.parse import urlparse
from fkie_master_discovery.common import masteruri_from_master


NMD_SERVER_PORT_OFFSET = 1010
''':var NMD_SERVER_PORT_OFFSET: offset to the ROS-Master port.'''


def equal_uri(url1, url2):
    '''
    Removes to string after remove last slash character.
    '''
    return url1.rstrip(os.path.sep) == url2.rstrip(os.path.sep)


def nmduri(uri='', prefix='grpc://'):
    '''
    Determine for given url a gRPC-URI with `grpc://` scheme. If the
    given URI is a ROS-Master URI the method calculate new port by adding
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
        muri = masteruri_from_master(True)
    o = urlparse(muri)
    port = o.port
    if o.scheme not in ['http', 'grpc']:
        raise ValueError("uri parameter does not contain a scheme of ['http', ''grpc']: %s" % uri)
    if o.scheme == 'http':
        port += NMD_SERVER_PORT_OFFSET
    return "%s%s:%d" % (prefix, o.hostname, port)


def masteruri(grpc_path):
    '''
    Determine ROS-Master uri from gRPC-URI by replacing the scheme and reducing the
    port by :const:`NMD_SERVER_PORT_OFFSET`.

    :param str grpc_path: an URI with `grpc://` scheme.
    :return: ROS-Master URI
    :rtype: str
    :raise ValueError: if uri is not empty and does not start with 'grpc://'.
    '''
    if not grpc_path:
        return masteruri_from_master(True)
    if not grpc_path.startswith('grpc://'):
        raise ValueError("Invalid grpc path to get masteruri: %s; `grpc` scheme missed!" % grpc_path)
    o = urlparse(grpc_path)
    port = o.port
    if o.scheme == 'grpc':
        port -= NMD_SERVER_PORT_OFFSET
    return "http://%s:%d/" % (o.hostname, port)


def nmdport(uri=''):
    '''
    Determine the port for GRPC-server from given URI. If empty try to get the ROS-Master URI.
    '''
    muri = uri
    if not muri:
        muri = masteruri_from_master(True)
    o = urlparse(muri)
    port = o.port
    if o.scheme == 'http':
        port += NMD_SERVER_PORT_OFFSET
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

    :param str masteruri: ROS-Master URI
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
