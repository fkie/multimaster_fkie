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


import re
import os
import platform
import socket
import sys
import threading
from urllib.parse import urlparse
from typing import List
from typing import Union

# cache for performance reasons
_local_addrs = None

ROS_IP = "ROS_IP"
ROS_IPV6 = "ROS_IPV6"
ROS_HOSTNAME = "ROS_HOSTNAME"
IP4_PATTERN = re.compile(r"^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}")

HOSTS_CACHE = dict()
''' :var HOSTS_CACHE: the cache directory to store the results of tests for local hosts. :meth:`is_local` '''

_LOCK = threading.RLock()


def get_hostname(url: str) -> Union[str, None]:
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


def subdomain(hostname: str) -> Union[str, None]:
    '''
    :return: the name with first subdomain
    '''
    if hostname is None:
        return None
    if IP4_PATTERN.match(hostname):
        return hostname
    return hostname.split('.')[0]


def ros_host_suffix(hostname: str='') -> str:
    '''
    Creates a suffix from hostname. If hostname is empty use the result of :meth:get_host_name().
    '''
    addr = hostname
    if not addr:
        addr = get_host_name()
    addr = subdomain(addr)
    addr = addr.replace('.', '_')
    return addr


def get_ros_hostname(url: str, host: str = None) -> str:
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


def is_local(hostname: str, wait: bool=True) -> bool:
    '''
    Test whether the given host name is the name of the local host or not.

    :param str hostname: the name or IP of the host
    :return: `True` if the hostname is local or None
    :rtype: bool
    :raise Exception: on errors while resolving host
    '''
    if not hostname:
        return True
    global HOSTS_CACHE
    global _LOCK
    with _LOCK:
        if hostname in HOSTS_CACHE:
            if isinstance(HOSTS_CACHE[hostname], threading.Thread):
                return False
            return HOSTS_CACHE[hostname]
    try:
        socket.inet_aton(hostname)
        local_addresses = ['localhost'] + get_local_addresses()
        # check 127/8 and local addresses
        result = hostname.startswith(
            '127.') or hostname == '::1' or hostname in local_addresses
        with _LOCK:
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


def __is_local(hostname: str) -> bool:
    '''
    Test the hostname whether it is local or not. Uses socket.gethostbyname().
    '''
    global HOSTS_CACHE
    global _LOCK
    try:
        # If Python has ipv6 disabled but machine.address can be resolved somehow to an ipv6 address, then host[4][0] will be int
        machine_ips = [host[4][0] for host in socket.getaddrinfo(
            hostname, 0, 0, 0, socket.SOL_TCP) if isinstance(host[4][0], str)]
    except socket.gaierror:
        with _LOCK:
            #Log.debug("host::HOSTS_CACHE resolve %s failed" % hostname)
            HOSTS_CACHE[hostname] = False
        return False
    local_addresses = ['localhost'] + get_local_addresses()
    # check 127/8 and local addresses
    result = ([ip for ip in machine_ips if (ip.startswith('127.') or ip == '::1')] != [
    ]) or (set(machine_ips) & set(local_addresses) != set())
    with _LOCK:
        #Log.debug("host::HOSTS_CACHE add %s:%s" % (hostname, result))
        HOSTS_CACHE[hostname] = result
    return result


def get_address_override() -> Union[str, None]:
    """
    :returns: ROS_IP/ROS_HOSTNAME override or None, ``str``
    :raises: :exc:`ValueError` If ROS_IP/ROS_HOSTNAME/__ip/__hostname are invalidly specified
    """
    # #998: check for command-line remappings first
    # TODO IPV6: check for compatibility
    for arg in sys.argv:
        if arg.startswith('__hostname:=') or arg.startswith('__ip:='):
            try:
                _, val = arg.split(':=')
                return val
            except:  # split didn't unpack properly
                raise ValueError(
                    "invalid ROS command-line remapping argument '%s'" % arg)

    # check ROS_HOSTNAME and ROS_IP environment variables, which are
    # aliases for each other
    if ROS_HOSTNAME in os.environ:
        hostname = os.environ[ROS_HOSTNAME]
        if hostname == '':
            raise ValueError('invalid ROS_HOSTNAME (an empty string)')
        else:
            parts = urlparse(hostname)
            if parts.scheme:
                msg = 'invalid ROS_HOSTNAME (protocol ' + (
                    'and port ' if parts.port else '') + 'should not be included)'
                raise ValueError('invalid ROS_HOSTNAME (protocol ' +
                                 ('and port ' if parts.port else '') + 'should not be included)')
            elif hostname.find(':') != -1:
                # this can not be checked with urlparse()
                # since it does not extract the port for a hostname like "foo:1234"
                raise ValueError(
                    'invalid ROS_HOSTNAME (port should not be included)')
        return hostname
    elif ROS_IP in os.environ:
        ip = os.environ[ROS_IP]
        if ip == '':
            raise ValueError('invalid ROS_IP (an empty string)')
        elif ip.find('://') != -1:
            raise ValueError(
                'invalid ROS_IP (protocol should not be included)')
        elif ip.find('.') != -1 and ip.rfind(':') > ip.rfind('.'):
            raise ValueError('invalid ROS_IP (port should not be included)')
        elif ip.find('.') == -1 and ip.find(':') == -1:
            raise ValueError(
                'invalid ROS_IP (must be a valid IPv4 or IPv6 address)')
        return ip
    return None


# def is_local_address(hostname):
#     """
#     :param hostname: host name/address, ``str``
#     :returns True: if hostname maps to a local address, False otherwise. False conditions include invalid hostnames.
#     """
#     try:
#         if use_ipv6():
#             reverse_ips = [host[4][0] for host in socket.getaddrinfo(hostname, 0, 0, 0, socket.SOL_TCP)]
#         else:
#             reverse_ips = [host[4][0] for host in socket.getaddrinfo(hostname, 0, socket.AF_INET, 0, socket.SOL_TCP)]
#     except socket.error:
#         return False
#     local_addresses = ['localhost'] + get_local_addresses()
#     # 127. check is due to #1260
#     if ([ip for ip in reverse_ips if (ip.startswith('127.') or ip == '::1')] != []) or (set(reverse_ips) & set(local_addresses) != set()):
#         return True
#     return False


def get_local_address() -> str:
    """
    :returns: default local IP address (e.g. eth0). May be overriden by ROS_IP/ROS_HOSTNAME/__ip/__hostname, ``str``
    """
    override = get_address_override()
    if override:
        return override
    addrs = get_local_addresses()
    if len(addrs) == 1:
        return addrs[0]
    for addr in addrs:
        # pick first non 127/8 address
        if not addr.startswith('127.') and not addr == '::1':
            return addr
    else:  # loopback
        if use_ipv6():
            return '::1'
        else:
            return '127.0.0.1'


def get_local_addresses() -> List[str]:
    """
    :returns: known local addresses. Not affected by ROS_IP/ROS_HOSTNAME, ``[str]``
    """
    # cache address data as it can be slow to calculate
    global _local_addrs
    if _local_addrs is not None:
        return _local_addrs

    local_addrs = None
    if platform.system() in ['Linux', 'FreeBSD', 'Darwin']:
        # unix-only branch
        v4addrs = []
        v6addrs = []
        import netifaces
        for iface in netifaces.interfaces():
            try:
                ifaddrs = netifaces.ifaddresses(iface)
            except ValueError:
                # even if interfaces() returns an interface name
                # ifaddresses() might raise a ValueError
                # https://bugs.launchpad.net/ubuntu/+source/netifaces/+bug/753009
                continue
            if socket.AF_INET in ifaddrs:
                v4addrs.extend([addr['addr']
                                for addr in ifaddrs[socket.AF_INET]])
            if socket.AF_INET6 in ifaddrs:
                v6addrs.extend([addr['addr']
                                for addr in ifaddrs[socket.AF_INET6]])
        if use_ipv6():
            local_addrs = v6addrs + v4addrs
        else:
            local_addrs = v4addrs
    else:
        # cross-platform branch, can only resolve one address
        if use_ipv6():
            local_addrs = [host[4][0] for host in socket.getaddrinfo(
                socket.gethostname(), 0, 0, 0, socket.SOL_TCP)]
        else:
            local_addrs = [host[4][0] for host in socket.getaddrinfo(
                socket.gethostname(), 0, socket.AF_INET, 0, socket.SOL_TCP)]
    _local_addrs = local_addrs
    return local_addrs


def use_ipv6() -> bool:
    return ROS_IPV6 in os.environ and os.environ[ROS_IPV6] == 'on'


# #528: semi-complicated logic for determining XML-RPC URI
def get_host_name() -> Union[str, None]:
    """
    Determine host-name for use in host-name-based addressing (e.g. XML-RPC URIs):
     - if ROS_IP/ROS_HOSTNAME is set, use that address
     - if the hostname returns a non-localhost value, use that
     - use whatever L{get_local_address()} returns
    """
    hostname = get_address_override()
    if not hostname:
        try:
            hostname = socket.gethostname()
        except:
            pass
        if not hostname or hostname == 'localhost' or hostname.startswith('127.'):
            hostname = get_local_address()
    return hostname
