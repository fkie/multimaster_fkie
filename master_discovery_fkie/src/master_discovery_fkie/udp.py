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

import array
import errno
import fcntl
import os
import platform
import socket
import struct
import threading

import roslib.network
import rospy


class DiscoverSocket(socket.socket):
    '''
    The DiscoverSocket class enables the send and receive UDP messages to a
    multicast group and unicast address. The unicast socket is only created if
    'unicast_only' parameter is set to True or a specific interface is defined.

    :param port: the port to bind the socket

    :type port: int

    :param mgroup: the multicast group to join

    :type mgroup: str

    :param reuse: allows the reusing of the port

    :type reuse: boolean (Default: True)

    :param ttl: time to leave

    :type ttl: int (Default: 20)
    '''

    def __init__(self, port, mgroup, ttl=20, unicast_only=False):
        '''
        Creates a socket, bind it to a given port and join to a given multicast
        group. IPv4 and IPv6 are supported.
        @param port: the port to bind the socket
        @type port: int
        @param mgroup: the multicast group to join
        @type mgroup: str
        @param ttl: time to leave
        @type ttl: int (Default: 20)
        @param unicast_only: send only unicast messages
        @type unicast_only: bool (Default: False)
        '''
        self.port = port
        self._lock = threading.RLock()
        self.unicast_only = unicast_only
        self._closed = False
        self._msg_callback = None
        self.sock_5_error_printed = []
        # get the group and interface. Especially for definition like 226.0.0.0@192.168.101.10
        # it also reads ~interface and ROS_IP
        self.mgroup, self.interface = DiscoverSocket.normalize_mgroup(mgroup, True)
        self.unicast_socket = None
        # get the AF_INET information for group to ensure that the address family
        # of group is the same as for interface
        addrinfo = UcastSocket.getaddrinfo(self.mgroup)
        self.interface_ip = ''
        if unicast_only:
            # inform about no braodcasting
            rospy.logwarn("Multicast disabled!")
        if self.interface:
            addrinfo = UcastSocket.getaddrinfo(self.interface, addrinfo[0])
            if addrinfo is not None:
                self.interface_ip = addrinfo[4][0]
                self.unicast_socket = UcastSocket(self.interface_ip, port)
        elif unicast_only:
            self.unicast_socket = UcastSocket('', port)

        rospy.logdebug("mgroup: %s", self.mgroup)
        rospy.logdebug("interface : %s", self.interface)
        rospy.logdebug("inet: %s", addrinfo)

        socket.socket.__init__(self, addrinfo[0], socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        if not unicast_only:
            rospy.loginfo("Listen for multicast at ('%s', %d)", self.mgroup, port)
            # initialize multicast socket
            # Allow multiple copies of this program on one machine
            if hasattr(socket, "SO_REUSEPORT"):
                try:
                    self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except:
                    rospy.logwarn("SO_REUSEPORT failed: Protocol not available, some functions are not available.")
            # Set Time-to-live (optional) and loop count
            ttl_bin = struct.pack('@i', ttl)
            if addrinfo[0] == socket.AF_INET:  # IPv4
                self.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl_bin)
                self.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
            else:  # IPv6
                self.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_HOPS, ttl_bin)
                self.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_MULTICAST_LOOP, 1)

            try:
                if addrinfo[0] == socket.AF_INET:  # IPv4
                    # Create group_bin for de-register later
                    # Set socket options for multicast specific interface or general
                    if not self.interface_ip:
                        self.group_bin = socket.inet_pton(socket.AF_INET, self.mgroup) + struct.pack('=I', socket.INADDR_ANY)
                        self.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                        self.group_bin)
                    else:
                        self.group_bin = socket.inet_aton(self.mgroup) + socket.inet_aton(self.interface_ip)
                        self.setsockopt(socket.IPPROTO_IP,
                                        socket.IP_MULTICAST_IF,
                                        socket.inet_aton(self.interface_ip))
                        self.setsockopt(socket.IPPROTO_IP,
                                        socket.IP_ADD_MEMBERSHIP,
                                        self.group_bin)
                else:  # IPv6
                    # Create group_bin for de-register later
                    # Set socket options for multicast
                    self.group_bin = socket.inet_pton(addrinfo[0], self.mgroup) + struct.pack('@I', socket.INADDR_ANY)
                    self.setsockopt(socket.IPPROTO_IPV6,
                                    socket.IPV6_JOIN_GROUP,
                                    self.group_bing)
            except socket.error as errobj:
                msg = str(errobj)
                if errobj.errno in [errno.ENODEV]:
                    msg = "socket.error[%d]: %s,\nis multicast route set? e.g. sudo route add -net 224.0.0.0 netmask 224.0.0.0 eth0" % (errobj.errno, msg)
                raise Exception(msg)

            # Bind to the port
            try:
                # bind to default interfaces if not unicast socket was created
                to_group = self.mgroup if self.unicast_socket is not None else ''
                self.bind((to_group, port))
            except socket.error as errobj:
                msg = str(errobj)
                rospy.logfatal("Unable to bind multicast to interface: %s, check that it exists: %s",
                               self.mgroup, msg)
                raise

        self.addrinfo = addrinfo
        if not unicast_only:
            # create a thread to handle the received multicast messages
            self._recv_thread = threading.Thread(target=self.recv_loop_multicast)
            self._recv_thread.setDaemon(True)
            self._recv_thread.start()
        if self.unicast_socket is not None:
            # create a thread to handle the received unicast messages
            self._recv_thread = threading.Thread(target=self.recv_loop_unicast)
            self._recv_thread.setDaemon(True)
            self._recv_thread.start()

    def set_message_callback(self, callback):
        '''
        This callback methos is called if the message is recieved.
        Callback API. (message, tuple of sender address)
        '''
        self._msg_callback = callback

    @staticmethod
    def normalize_mgroup(mgroup, getinterface=False):
        groupaddr, _, interface = mgroup.partition('@')
        if not getinterface:
            return groupaddr
        # Interface parameter takes precedence
        if not interface:
            interface = rospy.get_param('~interface', '')
        if interface:
            return groupaddr, interface
        # Otherwise, resolve interface with ROS_HOSTNAME or ROS_IP as per:
        # http://wiki.ros.org/ROS/EnvironmentVariables#ROS_IP.2BAC8-ROS_HOSTNAME
        if 'ROS_HOSTNAME' in os.environ:
            addr = socket.gethostbyname(os.environ['ROS_HOSTNAME'])
            if addr[:4] != '127.':  # 127.x.y.z is loopback
                return groupaddr, addr
        if 'ROS_IP' in os.environ:
            return groupaddr, os.environ['ROS_IP']
        # Cannot determine network interface
        return groupaddr, None

    def close(self):
        '''
        Unregister from the multicast group and close the socket.
        '''
        self._closed = True
        # Use the stored group_bin to de-register
        if not self.unicast_only:
            if self.addrinfo[0] == socket.AF_INET:  # IPv4
                self.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, self.group_bin)
            else:  # IPv6
                self.setsockopt(socket.IPPROTO_IPV6,
                                socket.IPV6_LEAVE_GROUP,
                                self.group_bing)
            rospy.loginfo("Stop receiving multicast at ('%s', %s)", self.mgroup, self.port)
            socket.socket.close(self)
        # close the unicast socket
        if self.unicast_socket is not None:
            self.unicast_socket.close()

    def send2group(self, msg):
        '''
        Sends the given message to the joined multicast group. Some errors on send
        will be ignored (``ENETRESET``, ``ENETDOWN``, ``ENETUNREACH``)

        :param msg: message to send

        :type msg: str
        '''
        try:
            if self.unicast_only and self.unicast_socket:
                self.unicast_socket.send2addr(msg, self.unicast_socket.interface)
            else:
                # Send to the multicast group address as supplied
                # Default '226.0.0.0'
                self.sendto(msg, (self.mgroup, self.getsockname()[1]))
        except socket.error as errobj:
            msg = str(errobj)
            if errobj.errno not in [errno.ENETDOWN, errno.ENETUNREACH, errno.ENETRESET]:
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
            if self.unicast_socket is None:
                self.sendto(msg, (addr, self.getsockname()[1]))
            else:
                self.unicast_socket.send2addr(msg, addr)
        except socket.error as errobj:
            msg = str(errobj)
            if errobj.errno in [-5]:
                if addr not in self.sock_5_error_printed:
                    rospy.logwarn("socket.error[%d]: %s, addr: %s", errobj.errno, msg, addr)
                    self.sock_5_error_printed.append(addr)
            elif errobj.errno not in [100, 101, 102]:
                raise

    def hasEnabledMulticastIface(self):
        '''
        Test all enabled interfaces for a MULTICAST flag. If no enabled interfaces
        has a multicast support, False will be returned.

        :return: ``True``, if any interfaces with multicast support are enabled.

        :rtype: bool
        '''
        SIOCGIFFLAGS = 0x8913
        IFF_MULTICAST = 0x1000  # Supports multicast.
        IFF_UP = 0x1  # Interface is up.
        for (ifname, _) in DiscoverSocket.localifs():
            args = (ifname + '\0' * 32)[:32]
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
        outbytes = struct.unpack('iL', fcntl.ioctl(sockfd.fileno(),
                                                   SIOCGIFCONF,
                                                   struct.pack('iL', MAXBYTES, names.buffer_info()[0])
                                                   ))[0]
        namestr = names.tostring()
        return [(namestr[i:i + var1].split('\0', 1)[0], socket.inet_ntoa(namestr[i + 20:i + 24]))
                for i in xrange(0, outbytes, var2)]

    def recv_loop_multicast(self):
        '''
        This method handles the received multicast messages.
        '''
        while not rospy.is_shutdown() and not self._closed:
            try:
                (msg, address) = self.recvfrom(1024)
                with self._lock:
                    if self._msg_callback is not None and not rospy.is_shutdown() and not self._closed:
                        self._msg_callback(msg, address)
            except socket.timeout:
                pass
            except socket.error:
                import traceback
                rospy.logwarn("socket error: %s", traceback.format_exc())

    def recv_loop_unicast(self):
        '''
        This method handles the received unicast messages.
        '''
        if self.unicast_socket is not None:
            while not rospy.is_shutdown() and not self._closed:
                try:
                    (msg, address) = self.unicast_socket.recvfrom(1024)
                    with self._lock:
                        if self._msg_callback is not None and not rospy.is_shutdown() and not self._closed:
                            self._msg_callback(msg, address)
                except socket.timeout:
                    pass
                except socket.error:
                    import traceback
                    rospy.logwarn("unicast socket error: %s", traceback.format_exc())


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
        self.interface = interface
        addrinfo = None
        # If interface isn't specified, try to find an non localhost interface to
        # get some info for binding. Otherwise use localhost
        if not self.interface:
            ifaces = roslib.network.get_local_addresses()
            self.interface = 'localhost'
            for iface in ifaces:
                if not (iface.startswith('127') or iface.startswith('::1')):
                    self.interface = iface
                    break
            rospy.loginfo("+ Bind to unicast socket @(%s:%s)", self.interface, port)
            addrinfo = UcastSocket.getaddrinfo(self.interface)
        else:
            # Otherwise get the address info for the interface specified.
            rospy.loginfo("+ Bind to specified unicast socket @(%s:%s)", self.interface, port)
            addrinfo = UcastSocket.getaddrinfo(self.interface)

        # Configure socket type
        socket.socket.__init__(self, addrinfo[0], socket.SOCK_DGRAM, socket.IPPROTO_UDP)

        # Allow multiple copies of this program on one machine
        # Required to receive unicast UDP
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except:
                rospy.logwarn("SO_REUSEPORT failed: Protocol not available, some functions are not available.")

        # Bind to the port
        try:
            rospy.logdebug("Ucast bind to: (%s:%s)", addrinfo[4][0], port)
            self.bind((addrinfo[4][0], port))
        except socket.error as errobj:
            msg = str(errobj)
            rospy.logfatal("Unable to bind unicast to interface: %s, check that it exists: %s",
                           self.interface, msg)
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
        except socket.error as errobj:
            msg = str(errobj)
            if errobj.errno in [-5]:
                if addr not in self.sock_5_error_printed:
                    rospy.logwarn("socket.error[%d]: %s, addr: %s", errobj.errno, msg, addr)
                    self.sock_5_error_printed.append(addr)
            elif errobj.errno in [errno.EINVAL, -2]:
                raise Exception('Cannot send to `%s`, try to change the interface, message: %s' % (addr, msg))
            elif errobj.errno not in [errno.ENETDOWN, errno.ENETUNREACH, errno.ENETRESET]:
                raise

    def close(self):
        """ Cleanup and close the socket"""
        socket.socket.close(self)

    @staticmethod
    def getaddrinfo(addr, family=None):
        '''
        :param addr: the addess to get the info for
        :param family: type of the address family (e.g. socket.AF_INET)
        '''
        # get info about the IP version (4 or 6)
        addrinfos = socket.getaddrinfo(addr, None)
        addrinfo = None
        if family is None and len(addrinfos) > 0:
            addrinfo = addrinfos[0]
        elif family:
            for ainfo in addrinfos:
                if ainfo[0] == family:
                    return ainfo
        return addrinfo
