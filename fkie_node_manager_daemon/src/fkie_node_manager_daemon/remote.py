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

from __future__ import division, absolute_import, print_function, unicode_literals

import os
import grpc
import rospy

from . import host

INSECURE_CHANNEL_CACHE = dict()
''' the cache for channels '''

try:
    del os.environ['https_proxy']
except Exception:
    pass
try:
    del os.environ['http_proxy']
except Exception:
    pass

# CREDENTIALS = ''
# # read in certificate
# with open('/home/tiderko/grpc_cert/server.crt', 'rb') as f:
#     trusted_certs = f.read()
#     # create credentials
#     CREDENTIALS = grpc.ssl_channel_credentials(root_certificates=trusted_certs)


class ChannelName:

    def __init__(self, url, hostname=''):
        self.url = url
        self.hostname = host.get_hostname(url)
        self.__hash = hash(url)

    def __eq__(self, other):
        return self.__hash == other.__hash

    def __hash__(self):
        return self.__hash


def clear_channels():
    global INSECURE_CHANNEL_CACHE
    INSECURE_CHANNEL_CACHE.clear()


def add_insecure_channel(url):
    '''
    Adds a new insecure channel for given url. Ports are ignored!
    :param str url: the url to parse
    '''
    global INSECURE_CHANNEL_CACHE
#     global CREDENTIALS
    cn = ChannelName(url)
    if cn not in INSECURE_CHANNEL_CACHE:
        rospy.logdebug("add insecure channel to %s" % url)
#         INSECURE_CHANNEL_CACHE[cn] = grpc.secure_channel(url, CREDENTIALS)
        INSECURE_CHANNEL_CACHE[cn] = grpc.insecure_channel(url)


def remove_insecure_channel(url):
    global INSECURE_CHANNEL_CACHE
    try:
        del INSECURE_CHANNEL_CACHE[url]
        rospy.logdebug("insecure channel to %s closed!" % url)
    except Exception:
        pass


def get_insecure_channel(url):
    '''
    :param str url: the url to parse or hostname. If hostname the channel should be added before.
    :return: returns insecure channel for given url. Ports are ignored!
    :rtype: grpc.Channel or None
    '''
    global INSECURE_CHANNEL_CACHE
#     global CREDENTIALS
    if url:
        cn = ChannelName(url)
        try:
            return INSECURE_CHANNEL_CACHE[cn]
        except Exception:
            if host.get_port(url):
                rospy.logdebug("create insecure channel to %s" % url)
                INSECURE_CHANNEL_CACHE[cn] = grpc.insecure_channel(url)
#                 INSECURE_CHANNEL_CACHE[cn] = grpc.secure_channel(url, CREDENTIALS)
                return INSECURE_CHANNEL_CACHE[cn]
    print("No cached URL for insecure channel: %s" % url)
    return None
