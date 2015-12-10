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
import socket
import rospy

class MasterEntry(object):

  def __init__(self, masteruri=None, mastername=None, address=None, hostname=None):
    self.masteruri = masteruri
    self._masternames = [mastername] if mastername else []
    self._hostnames = [hostname] if hostname else []
    self.mutex = RLock()
    self._addresses = []
    self._resolves = {}  # hostname : address
    self.addAddress(address)

  def __repr__(self):
    return ''.join([str(self.masteruri), ':\n',
                   '  masternames: ', str(self._masternames), '\n',
                   '  hostnames: ', str(self._hostnames), '\n',
                   '  addresses: ', str(self._addresses), '\n'])

  def hasMastername(self, mastername):
    if mastername in self._masternames:
      return True
    return False

  def hasHostname(self, hostname):
    if hostname in self._hostnames:
      return True
    return False

  def hasAddress(self, address):
    if address in self._addresses:
      return True
    return False

  def addMastername(self, mastername):
    if mastername and not self.hasMastername(mastername):
      self._masternames.append(mastername)

  def addHostname(self, hostname):
    if hostname and not self.hasHostname(hostname):
      self._hostnames.append(hostname)

  def addAddress(self, address):
    with self.mutex:
      host = address
      if address in self._resolves:
        host = self._resolves[address]
      if host and not self.hasAddress(host):
        self._add_address(host)

  def _add_address(self, address):
    host = address
    if address in self._resolves:
      host = self._resolves[address]
    try:
      socket.inet_pton(socket.AF_INET, host)
      # ok, it is a legal IPv4 address
      with self.mutex:
        self._addresses.append(host)
    except socket.error:
      # try for IPv6
      try:
        socket.inet_pton(socket.AF_INET6, host)
        # ok, it is a legal IPv6 address
        with self.mutex:
          if not self.hasAddress(host):
            self._addresses.append(host)
      except socket.error:
        # not legal IP address, resolve the name in a thread
        thread = Thread(target=self._get_address, args=((address,)))
        thread.daemon = True
        thread.start()

  def _get_address(self, hostname):
    try:
      # resolve hostname
      addr_infos = socket.getaddrinfo(hostname, 0)
      ip4_addr = ''
      ip6_addr = ''
      for (family, socktype, _, _, sockaddr) in addr_infos:
        if socktype == socket.SOCK_DGRAM:
          if family == socket.AF_INET:
            ip4_addr = sockaddr[0]
          elif family == socket.AF_INET6:
            ip6_addr = sockaddr[0]
      # we prefer IPv4
      machine_addr = ip4_addr if ip4_addr else ip6_addr
      with self.mutex:
        self._resolves[hostname] = machine_addr
        if not self.hasAddress(machine_addr):
          self._addresses.append(machine_addr)
    except socket.gaierror:
      # no suitable address found
      pass

  def getMastername(self):
    try:
      return self._masternames[0]
    except:
      return None

  def getMasternames(self):
    return list(self._masternames)

  def getAddress(self):
    with self.mutex:
      try:
        return self._addresses[0]
      except:
        return None

  def getHostname(self):
    try:
      return self._hostnames[0]
    except:
      return None

  def removeExtMastername(self, mastername):
    try:
      self._masternames.remove(mastername)
    except:
      pass

  def removeExtHostname(self, hostname):
    try:
      self._hostnames.remove(hostname)
    except:
      pass

  def removeExtAddress(self, address):
    try:
      self._addresses.remove(address)
    except:
      pass


class NameResolution(object):
  '''
  This class stores the association between master URI, master name and 
  host name or IP. Both the setter and the getter methods are thread safe.
  '''

  def __init__(self):
    self.mutex = RLock()
    self._masters = [] #sets with masters
    self._hosts = [] #sets with hosts
    self._address = [] # avoid the mixing of ip and name as address

  def removeMasterEntry(self, masteruri):
    with self.mutex:
      for m in self._masters:
        if masteruri and m.masteruri == masteruri:
          self._masters.remove(m)
          return

  def removeInfo(self, mastername, hostname):
    with self.mutex:
      for m in self._masters:
        if m.hasMastername(mastername) and (m.hasHostname(hostname) or m.hasAddress(hostname)):
          m.removeExtMastername(mastername)
          m.removeExtHostname(hostname)
          m.removeExtAddress(hostname)
          return

#  def remove(self, name=None, masteruri=None, host=None):
#    '''
#    Remove an association. If one of the given parameter found in the association,
#    the association will be removed from the name resolution list.
#    @param name: the name of the ROS master.
#    @type name: C{str} (Default: C{None}) 
#    @param masteruri: the URI of the ROS master.
#    @type masteruri: C{str} (Default: C{None}) 
#    @param host: the host name or IP
#    @type host: C{str} (Default: C{None}) 
#    '''
#    try:
#      self.mutex.acquire()
#      for s in list(self._masters):
#        if (name, 'name') in s or (masteruri, 'uri') in s or (host, 'host') in s:
#          try:
#            s.remove((name, 'name'))
#          except:
#            pass
#          try:
#            s.remove((masteruri, 'uri'))
#          except:
#            pass
#          try:
#            s.remove((host, 'host'))
#            self._address.remove(host)
#          except:
#            pass
#          if len(s) < 2:
#            self._masters.remove(s)
#    finally:
#      self.mutex.release()

  def addMasterEntry(self, masteruri, mastername, address, hostname=None):
    with self.mutex:
      mastername = self._validateMastername(mastername, masteruri)
      for m in self._masters:
        if m.masteruri and m.masteruri == masteruri:
          m.addMastername(mastername)
          m.addHostname(hostname)
          m.addAddress(address)
          return
        elif m.masteruri is None and m.hasMastername(mastername):
          m.masteruri = masteruri
          m.addMastername(mastername)
          m.addHostname(hostname)
          m.addAddress(address)
          return
      self._masters.append(MasterEntry(masteruri, mastername, address, hostname))

  def addInfo(self, mastername, address, hostname=None):
    with self.mutex:
#      mastername = self._validateMastername(mastername)
      for m in self._masters:
        if m.hasMastername(mastername):
          m.addMastername(mastername)
          m.addHostname(hostname)
          m.addAddress(address)
          return
        elif mastername is None:
          if m.hasHostname(hostname) or m.hasAddress(address):
            m.addHostname(hostname)
            m.addAddress(address)
      if not mastername is None:
        self._masters.append(MasterEntry(None, mastername, address, hostname))

  def _validateMastername(self, mastername, masteruri):
    '''
    Not thead safe
    '''
    mm = self.masteruri(mastername)
    if mm and mm != masteruri:
      nr = 2
      new_name = '%s_%d' % (mastername, nr)
      while mm and mm != masteruri:
        new_name = '%s_%d' % (mastername, nr)
        nr = nr + 1
      rospy.logwarn("master name '%s' is already assigned to '%s', rename to '%s'"(mastername, mm, new_name))
      return new_name
    return mastername

  def hasMaster(self, masteruri):
    with self.mutex:
      for m in self._masters:
        if m.masteruri == masteruri:
          return True
      return False

  def mastername(self, masteruri, address=None):
    with self.mutex:
      for m in self._masters:
        if m.masteruri == masteruri:
          if not address is None:
            if m.hasAddress(address):
              return m.getMastername()
          else:
            return m.getMastername()
      return None

  def masternames(self, masteruri):
    with self.mutex:
      for m in self._masters:
        if m.masteruri == masteruri:
          return m.getMasternames()
      return list()

  def masternameByAddress(self, address):
    with self.mutex:
      for m in self._masters:
        if m.hasAddress(address):
          return m.getMastername()
      return None

  def masteruri(self, mastername):
    with self.mutex:
      for m in self._masters:
        if m.hasMastername(mastername):
          return m.masteruri
      return None

  def masterurisByHost(self, hostname):
    with self.mutex:
      result = []
      for m in self._masters:
        if m.hasHostname(hostname) and m.masteruri and not m.masteruri in result:
          result.append(m.masteruri)
      return result

  def address(self, masteruri):
    with self.mutex:
      for m in self._masters:
        if m.masteruri == masteruri or m.hasHostname(masteruri) or m.hasMastername(masteruri):
          return m.getAddress()
      return None

  def hostname(self, address):
    with self.mutex:
      for m in self._masters:
        if m.hasAddress(address) or m.hasMastername(address):
          result = m.getHostname()
          return result if result else address
      return address

  @classmethod
  def getHostname(cls, url):
    '''
    Returns the host name used in a url

    @return: host or None if url is invalid
    @rtype:  C{str}
    '''
    if url is None:
      return None
    from urlparse import urlparse
    o = urlparse(url)
    return o.hostname
