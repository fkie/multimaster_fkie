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
import platform
import socket
import struct
import sys
import threading
try:
    from urlparse import urlparse  # python 2 compatibility
except ImportError:
    from urllib.parse import urlparse

import rospy
from rosgraph.network import get_local_addresses, get_local_address


HOSTS_CACHE = dict()
''' :var HOSTS_CACHE: the cache directory to store the results of tests for local hosts. :meth:`is_local` '''

_LOCK = threading.RLock()


def get_hostname(url):
    '''
    Extracts the hostname from given url.

    :param str url: the url to parse
    :return: the hostname or `None`, if the url is `None` or `invalid`
    :rtype: str
    :ref: http://docs.python.org/library/urlparse.html
    '''
    if url is None:
        return None
    if not url:
        return url
    o = urlparse(url)
    hostname = o.hostname
    if hostname is None:
        res = url.split(':')
        if len(res) == 2:
            return res[0]
        return url
    return hostname


def get_port(url):
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


def get_ros_hostname(url, host=None):
    '''
    Returns the host name used in a url, if it is a name. If it is an IP an
    empty string will be returned.

    :return: host or '' if url is an IP or invalid
    :rtype:  str
    '''
    hostname = get_hostname(url)
    if hostname is not None:
        if 'localhost' not in [hostname, host]:
            if '.' not in hostname and ':' not in hostname:
                local_hostname = 'localhost'
                try:
                    # ROS resolves the 'localhost' to local hostname
                    local_hostname = socket.gethostname()
                except Exception:
                    pass
                if local_hostname not in [hostname, host]:
                    return hostname
    return ''


def is_local(hostname, wait=True):
    '''
    Test whether the given host name is the name of the local host or not.

    :param str hostname: the name or IP of the host
    :return: `True` if the hostname is local or None
    :rtype: bool
    :raise Exception: on errors while resolving host
    '''
    if not hostname:
        return True
    with _LOCK:
        if hostname in HOSTS_CACHE:
            if isinstance(HOSTS_CACHE[hostname], threading.Thread):
                return False
            return HOSTS_CACHE[hostname]
    try:
        socket.inet_aton(hostname)
        local_addresses = ['localhost'] + get_local_addresses()
        # check 127/8 and local addresses
        result = hostname.startswith('127.') or hostname == '::1' or hostname in local_addresses
        with _LOCK:
            rospy.logdebug("host::HOSTS_CACHE add local %s:%s" % (hostname, result))
            HOSTS_CACHE[hostname] = result
        return result
    except socket.error:
        # the hostname must be resolved => do it in a thread
        if wait:
            result = __is_local(hostname)
            return result
        else:
            thread = threading.Thread(target=__is_local, args=((hostname,)))
            thread.daemon = True
            with _LOCK:
                HOSTS_CACHE[hostname] = thread
            thread.start()
    return False


def __is_local(hostname):
    '''
    Test the hostname whether it is local or not. Uses socket.gethostbyname().
    '''
    try:
        # If Python has ipv6 disabled but machine.address can be resolved somehow to an ipv6 address, then host[4][0] will be int
        machine_ips = [host[4][0] for host in socket.getaddrinfo(hostname, 0, 0, 0, socket.SOL_TCP) if isinstance(host[4][0], str)]
    except socket.gaierror:
        with _LOCK:
            rospy.logdebug("host::HOSTS_CACHE resolve %s failed" % hostname)
            HOSTS_CACHE[hostname] = False
        return False
    local_addresses = ['localhost'] + get_local_addresses()
    # check 127/8 and local addresses
    result = ([ip for ip in machine_ips if (ip.startswith('127.') or ip == '::1')] != []) or (set(machine_ips) & set(local_addresses) != set())
    with _LOCK:
        rospy.logdebug("host::HOSTS_CACHE add %s:%s" % (hostname, result))
        HOSTS_CACHE[hostname] = result
    return result
