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
from urlparse import urlparse
from master_discovery_fkie.common import masteruri_from_master


NMD_SERVER_PORT_OFFSET = 1010


def equal_uri(url1, url2):
    return url1.rstrip(os.path.sep) == url2.rstrip(os.path.sep)


def get_nmd_url(uri='', prefix='grpc://'):
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


def get_masteruri_from_nmd(grpc_path):
    if not grpc_path:
        return masteruri_from_master(True)
    if not grpc_path.startswith('grpc://'):
        raise ValueError("Invalid grpc path to get masteruri: %s; `grpc` scheme missed!" % grpc_path)
    o = urlparse(grpc_path)
    port = o.port
    if o.scheme == 'grpc':
        port -= NMD_SERVER_PORT_OFFSET
    return "http://%s:%d" % (o.hostname, port)


def get_nmd_port(uri=''):
    muri = uri
    if not muri:
        muri = masteruri_from_master(True)
    o = urlparse(muri)
    port = o.port
    if o.scheme == 'http':
        port += NMD_SERVER_PORT_OFFSET
    return port


def grpc_create_url(masteruri, path):
    if path.startswith(os.path.sep) or not path:
        return "%s%s" % (get_nmd_url(masteruri), path)
    return "%s%s%s" % (get_nmd_url(masteruri), os.path.sep, path)


def grpc_url_from_path(grpc_path):
    url = grpc_path
    if not grpc_path:
        url = grpc_create_url('', '')
    if url and not url.startswith('grpc://'):
        raise ValueError("Invalid grpc path to split: %s; `grpc` scheme missed!" % grpc_path)
    url_parse_result = urlparse(url)
    return 'grpc://%s' % (url_parse_result.netloc)


def grpc_join(url, path):
    if not path.startswith('grpc://'):
        if not url.startswith('grpc://'):
            return grpc_create_url(url, path)
        if path.startswith(os.path.sep):
            return '%s%s' % (url, path)
        return '%s%s%s' % (url, os.path.sep, path)
    return path


def grpc_split_url(grpc_path, with_scheme=False):
    url = grpc_path
    if not grpc_path:
        url = get_nmd_url()
    if url and not url.startswith('grpc://'):
        raise ValueError("Invalid grpc path to split: %s; `grpc` scheme missed!" % grpc_path)
    url_parse_result = urlparse(url)
    if with_scheme:
        return ("%s://%s" % (url_parse_result.scheme, url_parse_result.netloc), url_parse_result.path)
    return (url_parse_result.netloc, url_parse_result.path)
