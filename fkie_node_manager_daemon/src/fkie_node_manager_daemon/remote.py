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
import grpc
import rospy
import time

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


def clear_channels():
    global INSECURE_CHANNEL_CACHE
    INSECURE_CHANNEL_CACHE.clear()


def remove_insecure_channel(url):
    global INSECURE_CHANNEL_CACHE
    try:
        del INSECURE_CHANNEL_CACHE[url]
        rospy.logdebug("insecure channel to %s closed!" % url)
    except Exception:
        pass


def get_insecure_channel(url, cached=False):
    '''
    :param str url: the url to parse or hostname. If hostname the channel should be added before.
    :return: returns insecure channel for given url
    :rtype: grpc.Channel or None
    '''
#     global CREDENTIALS
    starttime = time.time()
    if cached:
        global INSECURE_CHANNEL_CACHE
        if url in INSECURE_CHANNEL_CACHE:
            return INSECURE_CHANNEL_CACHE[url]
    rospy.logdebug("create insecure channel to %s" % url)
    channel = grpc.insecure_channel(url)
    if time.time() - starttime > 5.0:
        rospy.logwarn("Open insecure gRPC channel took too long (%.3f sec)! Fix your network configuration!" % (time.time() - starttime))
    if cached:
        INSECURE_CHANNEL_CACHE[url] = channel
    return channel
