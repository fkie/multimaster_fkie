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

import socket
import struct
import fcntl
import array
import platform
import errno

import rospy

class McastSocket(socket.socket):
  '''
  The McastSocket class enables the send and receive UDP messages to a multicast
  group.

  :param interface: The interface to bind to
  :type interface: string
  :param port: the port to bind the socket
  :type port: int
  :param mgroup: the multicast group to join
  :type mgroup: str
  :param ttl: time to leave
  :type ttl: int (Default: 20)
  '''

  def __init__(self, interface, port, mgroup, ttl=20):
    '''
    Creates a socket, bind it to a given port and join to a given multicast group.
    IPv4 and IPv6 are supported.

    @param port: the port to bind the socket
    @type port: int
    @param mgroup: the multicast group to join
    @type mgroup: str
    @param ttl: time to leave
    @type ttl: int (Default: 20)
    '''
    # Set multicast group for sending to multicast
    self.mgroup = mgroup
    self.interface = interface

    # Get AF_INET information for interface if specified.
    # Otherwise get the AF_INET information for group so we have something.
    addrinfo = None
    if not self.interface:
      addrinfo = McastSocket.getaddrinfo(mgroup)
    else:
      addrinfo = McastSocket.getaddrinfo(interface)

    socket.socket.__init__(self, addrinfo[0], socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Allow multiple copies of this program on one machine
    # Required to receive unicast UDP
    self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
      self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

    # Set Time-to-live (optional) and loop count
    ttl_bin = struct.pack('@i', ttl)
    if addrinfo[0] == socket.AF_INET: # IPv4
      self.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl_bin)
      self.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    else:# IPv6
      self.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, ttl_bin)
      self.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_LOOP, 1)

    #join to the multicast group
    rospy.logdebug("mgroup: %s", mgroup)
    rospy.logdebug("interface : %s", interface)
    rospy.logdebug("inet: %s", addrinfo)

    try:
      if addrinfo[0] == socket.AF_INET: # IPv4
        # Create group_bin for de-register later
        # Set socket options for multicast specific interface or general
        if not self.interface:
          self.group_bin = socket.inet_pton(socket.AF_INET, mgroup) + struct.pack('=I', socket.INADDR_ANY)
          self.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                          self.group_bin)
        else:
          self.group_bin = socket.inet_aton(mgroup) + socket.inet_aton(self.interface)
          self.setsockopt(socket.IPPROTO_IP,
                          socket.IP_MULTICAST_IF,
                          socket.inet_aton(self.interface))
          self.setsockopt(socket.IPPROTO_IP,
                          socket.IP_ADD_MEMBERSHIP,
                          self.group_bin)
      else: #IPv6
        # Create group_bin for de-register later
        # Set socket options for multicast
        self.group_bin = socket.inet_pton(addrinfo[0], mgroup) + struct.pack('@I', socket.INADDR_ANY)
        self.setsockopt(socket.IPPROTO_IPV6,
                        socket.IPV6_JOIN_GROUP,
                        self.group_bing)

    except socket.error, (errn, msg):
      err = str(msg)
      if errn in [19]:
        err = ''.join(["socket.error[", str(errn), "]: ", msg, ",\nis multicast route set? e.g. sudo route add -net 224.0.0.0 netmask 224.0.0.0 eth0"])
      raise Exception(err)

    # Bind to the port
    try:
      self.bind((mgroup, port))
    except socket.error, (errn, msg):
      err = str(msg)
      rospy.logfatal("Unable to bind to interface: %s, check that it exists: %s",
                     self.interface, err)
      raise

    self.addrinfo = addrinfo
    self.sock_5_error_printed = []

  @staticmethod
  def getaddrinfo(addr):
    # get info about the IP version (4 or 6)
    addrinfos = socket.getaddrinfo(addr, None)
    addrinfo = None
    if len(addrinfos) > 1:
      addrinfo = addrinfos[0]
    return addrinfo

  @staticmethod
  def normalize_mgroup(mgroup, getinterface=False):
    groupaddr, _, interface = mgroup.partition('@')
    if getinterface:
      return groupaddr, interface
    return groupaddr

  def close(self):
    '''
    Unregister from the multicast group and close the socket.
    '''
    # Use the stored group_bin to de-register
    if self.addrinfo[0] == socket.AF_INET: # IPv4
        self.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, self.group_bin)
    else: #IPv6
        self.setsockopt(socket.IPPROTO_IPV6,
                        socket.IPV6_LEAVE_GROUP,
                        self.group_bing)
    socket.socket.close(self)

  def send2group(self, msg):
    '''
    Sends the given message to the joined multicast group. Some errors on send
    will be ignored (``ENETRESET``, ``ENETDOWN``, ``ENETUNREACH``)

    :param msg: message to send
    :type msg: str
    '''
    try:
      # Send to the multicast group address as supplied
      # Default '226.0.0.0'
      self.sendto(msg, (self.mgroup, self.getsockname()[1]))
    except socket.error, (errn, msg):
      if not errn in [errno.ENETDOWN, errno.ENETUNREACH, errno.ENETRESET]:
        raise

  def hasEnabledMulticastIface(self):
    '''
    Test all enabled interfaces for a MULTICAST flag. If no enabled interfaces
    has a multicast support, False will be returned.

    :return: ``True``, if any interfaces with multicast support are enabled.

    :rtype: bool
    '''
    SIOCGIFFLAGS = 0x8913
    IFF_MULTICAST = 0x1000 # Supports multicast.
    IFF_UP = 0x1           # Interface is up.
    for (ifname, ip) in McastSocket.localifs():
      args = (ifname + '\0'*32)[:32]
      try:
        result = fcntl.ioctl(self.fileno(), SIOCGIFFLAGS, args)
      except IOError:
        return False
      flags, = struct.unpack('H', result[16:18])
      if ((flags & IFF_MULTICAST) != 0) & ((flags & IFF_UP) != 0):
        return True
    return False

  @staticmethod
  def localifs():
    '''
    Used to get a list of the up interfaces and associated IP addresses
    on this machine (linux only).

    :return:
        List of interface tuples.  Each tuple consists of
        ``(interface name, interface IP)``
    :rtype: list of ``(str, str)``
    '''
    SIOCGIFCONF = 0x8912
    MAXBYTES = 8096

    arch = platform.architecture()[0]

    # I really don't know what to call these right now
    var1 = -1
    var2 = -1
    if arch == '32bit':
      var1 = 32
      var2 = 32
    elif arch == '64bit':
      var1 = 16
      var2 = 40
    else:
      raise OSError("Unknown architecture: %s" % arch)

    sockfd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    names = array.array('B', '\0' * MAXBYTES)
    outbytes = struct.unpack('iL', fcntl.ioctl(
        sockfd.fileno(),
        SIOCGIFCONF,
        struct.pack('iL', MAXBYTES, names.buffer_info()[0])
        ))[0]

    namestr = names.tostring()
    return [(namestr[i:i+var1].split('\0', 1)[0], socket.inet_ntoa(namestr[i+20:i+24])) \
            for i in xrange(0, outbytes, var2)]


class UcastSocket(socket.socket):

  def __init__(self, interface, port):
    '''
    Creates a socket, bind it to a given interface+port for unicast send/receive.
    IPv4 and IPv6 are supported.

    @param interface: The interface to bind to
    @type interface: str
    @param port: the port to bind the socket
    @type port: int
    '''
    # Set multicast group for sending to multicast
    self.interface = interface

    # Get AF_INET information for interface if specified.
    # Otherwise get the AF_INET information for group so we have something.
    addrinfo = None

    # If interface isn't specified, use localhost to get some info for binding
    # (AF_INET etc...)
    if not self.interface:
      addrinfo = UcastSocket.getaddrinfo('localhost')
    else:
      # Otherwise get the address info for the interface specified.
      addrinfo = UcastSocket.getaddrinfo(interface)

    # Configure socket type
    socket.socket.__init__(self, addrinfo[0], socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Allow multiple copies of this program on one machine
    # Required to receive unicast UDP
    self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
      self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

    # Bind to the port
    try:
      rospy.logdebug("Ucast bind to: (%s:%s)", interface, port)
      self.bind((interface, port))
    except socket.error, (errn, msg):
      err = str(msg)
      rospy.logfatal("Unable to bind to interface: %s, check that it exists: %s",
                     self.interface, err)
      raise

  def send2addr(self, msg, addr):
    '''
    Sends the given message to the joined multicast group. Some errors on send
    will be ignored (``ENETRESET``, ``ENETDOWN``, ``ENETUNREACH``)

    :param msg: message to send
    :type msg: str
    :param addr: IPv4 or IPv6 address
    :type addr: str
    '''
    try:
      self.sendto(msg, (addr, self.getsockname()[1]))
    except socket.error, (errn, msg):
      if errn in [-5]:
        if not addr in self.sock_5_error_printed:
          rospy.logwarn("socket.error[%d]: %s, addr: %s", errn, msg, addr)
          self.sock_5_error_printed.append(addr)
      elif not errn in [errno.ENETDOWN, errno.ENETUNREACH, errno.ENETRESET]:
        raise

  def close(self):
    """ Cleanup and close the socket"""
    socket.socket.close()

  @staticmethod
  def getaddrinfo(addr):
    # get info about the IP version (4 or 6)
    addrinfos = socket.getaddrinfo(addr, None)
    addrinfo = None
    if len(addrinfos) > 1:
      addrinfo = addrinfos[0]
    return addrinfo
