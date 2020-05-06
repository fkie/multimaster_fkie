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

ROS_IP = 'ROS_IP'
ROS_HOSTNAME = 'ROS_HOSTNAME'
SIOCGIFCONF = 0x8912
SIOCGIFADDR = 0x8915
if platform.system() == 'FreeBSD':
    SIOCGIFADDR = 0xc0206921
    if platform.architecture()[0] == '64bit':
        SIOCGIFCONF = 0xc0106924
    else:
        SIOCGIFCONF = 0xc0086924

if 0:
    # disabling netifaces as it accounts for 50% of startup latency
    try:
        import netifaces
        _use_netifaces = True
    except Exception:
        # NOTE: in rare cases, I've seen Python fail to extract the egg
        # cache when launching multiple python nodes.  Thus, we do
        # except-all instead of except ImportError (kwc).
        _use_netifaces = False
else:
    _use_netifaces = False

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


# cache for performance reasons
_local_addrs = None


def get_local_addresses():
    """
    @return: known local addresses. Not affected by ROS_IP/ROS_HOSTNAME
    @rtype:  [str]
    """
    # cache address data as it can be slow to calculate
    global _local_addrs
    if _local_addrs is not None:
        return _local_addrs

    local_addrs = None
    if _use_netifaces:
        # #552: netifaces is a more robust package for looking up
        # #addresses on multiple platforms (OS X, Unix, Windows)
        local_addrs = []
        # see http://alastairs-place.net/netifaces/
        for i in netifaces.interfaces():
            try:
                local_addrs.extend([d['addr'] for d in netifaces.ifaddresses(i)[netifaces.AF_INET]])
            except KeyError:
                pass
    elif platform.system() in ['Linux', 'FreeBSD']:
        # unix-only branch
        # adapted from code from Rosen Diankov (rdiankov@cs.cmu.edu)
        # and from ActiveState recipe

        import fcntl
        import array

        ifsize = 32
        if platform.system() == 'Linux' and platform.architecture()[0] == '64bit':
            ifsize = 40  # untested

        # 32 interfaces allowed, far more than ROS can sanely deal with

        max_bytes = 32 * ifsize
        # according to http://docs.python.org/library/fcntl.html, the buffer limit is 1024 bytes
        buff = array.array('B', [0 for i in range(max_bytes)])
        # serialize the buffer length and address to ioctl
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        info = fcntl.ioctl(sock.fileno(), SIOCGIFCONF,
                           struct.pack('iL', max_bytes, buff.buffer_info()[0]))
        retbytes = struct.unpack('iL', info)[0]
        buffstr = buff.tostring()
        if platform.system() == 'Linux':
            local_addrs = [socket.inet_ntoa(buffstr[i+20:i+24]) for i in range(0, retbytes, ifsize)]
        else:
            # in FreeBSD, ifsize is variable: 16 + (16 or 28 or 56) bytes
            # When ifsize is 32 bytes, it contains the interface name and address,
            # else it contains the interface name and other information
            # This means the buffer must be traversed in its entirety
            local_addrs = []
            bufpos = 0
            while bufpos < retbytes:
                bufpos += 16
                ifreqsize = ord(buffstr[bufpos])
                if ifreqsize == 16:
                    local_addrs += [socket.inet_ntoa(buffstr[bufpos+4:bufpos+8])]
                bufpos += ifreqsize
    else:
        # cross-platform branch, can only resolve one address
        local_addrs = [socket.gethostbyname(socket.gethostname())]
    _local_addrs = local_addrs
    return local_addrs


def get_address_override():
    """
    @return: ROS_IP/ROS_HOSTNAME override or None
    @rtype: str
    @raise ValueError: if ROS_IP/ROS_HOSTNAME/__ip/__hostname are invalidly specified
    """
    # #998: check for command-line remappings first
    for arg in sys.argv:
        if arg.startswith('__hostname:=') or arg.startswith('__ip:='):
            try:
                _, val = arg.split(':=')
                return val
            except Exception:  # split didn't unpack properly
                raise ValueError("invalid ROS command-line remapping argument '%s'" % arg)

    # check ROS_HOSTNAME and ROS_IP environment variables, which are
    # aliases for each other
    if ROS_HOSTNAME in os.environ:
        return os.environ[ROS_HOSTNAME]
    elif ROS_IP in os.environ:
        return os.environ[ROS_IP]
    return None


def get_local_address():
    """
    @return: default local IP address (e.g. eth0). May be overriden by ROS_IP/ROS_HOSTNAME/__ip/__hostname
    @rtype: str
    """
    override = get_address_override()
    if override:
        return override
    addrs = get_local_addresses()
    if len(addrs) == 1:
        return addrs[0]
    for addr in addrs:
        # pick first non 127/8 address
        if not addr.startswith('127.'):
            return addr
    else:  # loopback
        return '127.0.0.1'
