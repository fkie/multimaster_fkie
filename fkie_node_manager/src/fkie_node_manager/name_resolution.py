# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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



from threading import Thread, RLock
try:
    from urlparse import urlparse  # python 2 compatibility
except ImportError:
    from urllib.parse import urlparse
import socket

import rospy
from fkie_master_discovery.common import get_hostname
from fkie_node_manager_daemon.common import utf8
from fkie_node_manager_daemon.common import isstring
from fkie_node_manager_daemon import url as nmdurl

RESOLVE_CACHE = {}  # hostname : address


class MasterEntry(object):

    def __init__(self, masteruri=None, mastername=None, address=None):
        self.masteruri = masteruri
        self._masternames = []
        if mastername:
            self.add_mastername(mastername)
        self.mutex = RLock()
        # addresses: hostname (at first place if available), IPv4 or IPv6
        self._addresses = []
        self.add_address(address)

    def __repr__(self):
        return "MasterEntry<%s, names=%s, addresses=%s>" % (self.masteruri, self._masternames, self._addresses)

    def entry(self):
        return [self.masteruri] + list(self._addresses)

    def has_mastername(self, mastername):
        return mastername in self._masternames

    def has_address(self, address):
        return address in self._addresses

    def add_mastername(self, mastername):
        if mastername and mastername not in self._masternames:
            self._masternames.insert(0, mastername)

    def add_address(self, address):
        with self.mutex:
            if address and not self.has_address(address):
                if self.is_legal_ip(address):
                    # it is an IP, try to get the hostname
                    self._addresses.append(address)
                    # resolve the name in a thread
                    thread = Thread(target=self._get_hostname, args=((address,)))
                    thread.daemon = True
                    thread.start()
                else:
                    # it is a hostname: add at the fist place and try to get an IP for this host
                    self._addresses.insert(0, address)
                    # resolve the name in a thread
                    thread = Thread(target=self._get_address, args=((address,)))
                    thread.daemon = True
                    thread.start()

    @classmethod
    def is_legal_ip(cls, addr):
        result = False
        try:
            socket.inet_pton(socket.AF_INET, addr)
            # ok, it is a legal IPv4 address
            result = True
        except socket.error:
            # try for IPv6
            try:
                socket.inet_pton(socket.AF_INET6, addr)
                # ok, it is a legal IPv6 address
                result = True
            except socket.error:
                # not legal IP address
                pass
        return result

    def _get_address(self, hostname):
        try:
            (_, _, ipaddrlist) = socket.gethostbyaddr(hostname)
            with self.mutex:
                if ipaddrlist:
                    RESOLVE_CACHE[hostname] = ipaddrlist
                    for addr in ipaddrlist:
                        if not self.has_address(addr):
                            self._addresses.append(addr)
        except Exception:
            # no suitable address found
            pass

    def _get_hostname(self, address):
        try:
            (hostname, _, _) = socket.gethostbyaddr(address)
            with self.mutex:
                name_splitted = hostname.split('.')
                RESOLVE_CACHE[address] = [name_splitted[0], hostname]
                if not self.has_address(hostname):
                    self._addresses.insert(0, hostname)
                if not self.has_address(name_splitted[0]):
                    self._addresses.insert(0, name_splitted[0])
        except Exception:
            # no suitable address found
            pass

    def get_mastername(self):
        try:
            return self._masternames[0]
        except Exception:
            return None

    def get_masternames(self):
        return list(self._masternames)

    def get_address(self, prefer_hostname=True):
        with self.mutex:
            try:
                if prefer_hostname:
                    return self._addresses[0]
                return self._addresses[-1]
            except Exception:
                return None

    def addresses(self):
        return list(self._addresses)

    def remove_mastername(self, mastername):
        try:
            self._masternames.remove(mastername)
        except Exception:
            pass

    def remove_address(self, address):
        try:
            self._addresses.remove(address)
        except Exception:
            pass

    def __eq__(self, item):
        if isinstance(item, MasterEntry):
            result = []
            if nmdurl.equal_uri(self.masteruri, item.masteruri):
                result = set(self.addresses()).intersection(set(item.addresses()))
            return len(result) > 0
        return False


class NameResolution(object):
    '''
    This class stores the association between master URI, master name and
    host name or IP. Both the setter and the getter methods are thread safe.
    '''

    def __init__(self):
        self.mutex = RLock()
        self._masters = []  # sets with masters
        self._hosts = []  # sets with hosts
        self._address = []  # avoid the mixing of ip and name as address

    def get_master(self, masteruri, address=None):
        me = MasterEntry(masteruri, None, address)
        with self.mutex:
            for m in self._masters:
                if m == me:
                    return m
        return me

    def remove_master_entry(self, masteruri):
        with self.mutex:
            for m in self._masters:
                if masteruri and m.masteruri == masteruri:
                    self._masters.remove(m)
                    return

    def remove_info(self, mastername, address):
        with self.mutex:
            for m in self._masters:
                if m.has_mastername(mastername) and m.has_address(address):
                    m.remove_mastername(mastername)
                    m.remove_address(address)
                    return

    def add_master_entry(self, masteruri, mastername, address):
        with self.mutex:
            mastername = self._validate_mastername(mastername, masteruri)
            for m in self._masters:
                if m.masteruri and m.masteruri == masteruri:
                    m.add_mastername(mastername)
                    m.add_address(address)
                    return
                elif m.masteruri is None and m.has_mastername(mastername):
                    m.masteruri = masteruri
                    m.add_mastername(mastername)
                    m.add_address(address)
                    return
            self._masters.append(MasterEntry(masteruri, mastername, address))

    def add_info(self, mastername, address):
        with self.mutex:
            for m in self._masters:
                if m.has_mastername(mastername):
                    m.add_mastername(mastername)
                    m.add_address(address)
                    return
            if mastername is not None:
                self._masters.append(MasterEntry(None, mastername, address))

    def _validate_mastername(self, mastername, masteruri):
        '''
        Not thread safe
        '''
        mm = self.masteruri(mastername)
        if mm and mm != masteruri:
            nr = 2
            new_name = '%s_%d' % (mastername, nr)
            mm = self.masteruri(new_name)
            while mm and mm != masteruri:
                nr = nr + 1
                new_name = '%s_%d' % (mastername, nr)
                mm = self.masteruri(new_name)
            rospy.logwarn("master name '%s' is already assigned to '%s', rename to '%s'" % (mastername, mm, new_name))
            return new_name
        return mastername

    def has_master(self, masteruri):
        with self.mutex:
            for m in self._masters:
                if m.masteruri == masteruri:
                    return True
            return False

    def mastername(self, masteruri, address=None):
        with self.mutex:
            for m in self._masters:
                if m.masteruri == masteruri:
                    if address is not None:
                        if m.has_address(address):
                            return m.get_mastername()
                    else:
                        return m.get_mastername()
            return get_hostname(masteruri)

    def masternames(self, masteruri):
        with self.mutex:
            for m in self._masters:
                if m.masteruri == masteruri:
                    return m.get_masternames()
            return list()

    def masternamebyaddr(self, address):
        with self.mutex:
            for m in self._masters:
                if m.has_address(address):
                    return m.get_mastername()
            return None

    def masteruri(self, mastername):
        with self.mutex:
            for m in self._masters:
                if m.has_mastername(mastername):
                    return m.masteruri
            return None

    def masteruribyaddr(self, address):
        with self.mutex:
            for m in self._masters:
                if m.has_address(address) and m.masteruri:
                    return m.masteruri
        return None

    def masterurisbyaddr(self, address):
        with self.mutex:
            result = []
            for m in self._masters:
                if m.has_address(address) and m.masteruri and m.masteruri not in result:
                    result.append(m.masteruri)
            return result

    def address(self, masteruri):
        with self.mutex:
            for m in self._masters:
                if m.masteruri == masteruri or m.has_mastername(masteruri):
                    return m.get_address(prefer_hostname=False)
            return get_hostname(masteruri)

    def addresses(self, masteruri):
        with self.mutex:
            for m in self._masters:
                if m.masteruri == masteruri or m.has_mastername(masteruri):
                    return m.addresses()
            return []

    def hostname(self, address, resolve=False):
        with self.mutex:
            for m in self._masters:
                if m.has_address(address) or m.has_mastername(address):
                    result = m.get_address()
                    if result and not MasterEntry.is_legal_ip(result):
                        return result
                    else:
                        break
        try:
            pass
            #self.add_address(address)
            # if MasterEntry.is_legal_ip(address):
            #     (hostname, _, _) = socket.gethostbyaddr(address)
            #     return hostname
        except Exception:
            import traceback
            print(traceback.format_exc())
        return address

    @classmethod
    def masteruri2name(cls, masteruri):
        result = masteruri
        try:
            url = urlparse(masteruri)
            if url.port == 11311:
                result = '%s' % url.hostname
            else:
                result = '%s_%d' % (url.hostname, url.port)
        except Exception:
            pass
        return cls.normalize_name(result)

    @classmethod
    def normalize_name(cls, name):
        result = name.replace('-', '_').replace('.', '_')
        return result

    @classmethod
    def is_legal_ip(cls, address):
        return MasterEntry.is_legal_ip(address)

    def resolve_cached(self, hostname):
        try:
            return RESOLVE_CACHE[hostname]
        except Exception:
            pass
        return [hostname]
