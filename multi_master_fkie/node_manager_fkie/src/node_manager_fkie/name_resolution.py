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
#  * Neither the name of I Heart Engineering nor the names of its
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

from threading import RLock

class NameResolution(object):
  '''
  This class stores the association between master URI, master name and 
  host name or IP. Both the setter and the getter methods are thread safe.
  '''
  
  def __init__(self):
    self.mutex = RLock()
    self._hosts = [] #sets with hosts
  
  def remove(self, name=None, masteruri=None, host=None):
    '''
    Remove an association. If one of the given parameter found in the association,
    the association will be removed from the name resolution list.
    @param name: the name of the ROS master.
    @type name: C{str} (Default: C{None}) 
    @param masteruri: the URI of the ROS master.
    @type masteruri: C{str} (Default: C{None}) 
    @param host: the host name or IP
    @type host: C{str} (Default: C{None}) 
    '''
    try:
      self.mutex.acquire()
      for s in list(self._hosts):
        if (name, 'name') in s or (masteruri, 'uri') in s or (host, 'host') in s:
          self._hosts.remove(s)
    finally:
      self.mutex.release()
  
  def add(self, name, masteruri=None, host=None):
    '''
    Adds a new association. 
    @param name: the name of the ROS master.
    @type name: C{str}
    @param masteruri: the URI of the ROS master.
    @type masteruri: C{str} (Default: C{None}) 
    @param host: the host name or IP
    @type host: C{str} (Default: C{None}) 
    '''
    try:
      self.mutex.acquire()
      added = False
      l = [(name, id) for name, id in [(name, 'name'), (masteruri, 'uri'), (host, 'host')] if not name is None]
      new_set = set(l)
      for s in self._hosts:
        if new_set&s:
          s |= new_set
          return
      self._hosts.append(new_set)
    finally:
      self.mutex.release()
    
  def getHost(self, name =None, masteruri=None):
    '''
    Returns for the name or masteruri the associated host name or ip, which is 
    first in the list.
    @return: the host name or ip
    @rtype: C{str} or C{None} 
    '''
    try:
      self.mutex.acquire()
      new_set = set([(name, id) for name, id in [(name, 'name'), (masteruri, 'uri')] if not name is None])
      for s in self._hosts:
        if new_set&s:
          for value, id in list(s):
            if id == 'host':
              return value
      return None
    finally:
      self.mutex.release()

  def getUri(self, host=None, name=None):
    '''
    Returns for the host or master name the associated masteruri, which is 
    first in the list.
    @return: the masteruri
    @rtype: C{str} or C{None} 
    '''
    try:
      self.mutex.acquire()
      new_set = set([(name, id) for name, id in [(name, 'name'), (host, 'host')] if not name is None])
      for s in self._hosts:
        if new_set&s:
          for value, id in list(s):
            if id == 'uri':
              return value
      return None
    finally:
      self.mutex.release()

  def getName(self, host=None, masteruri=None):
    '''
    Returns for the host or masteruri the associated name of the master , which is 
    first in the list.
    @return: the name of the master
    @rtype: C{str} or C{None} 
    '''
    try:
      self.mutex.acquire()
      new_set = set([(name, id) for name, id in [(masteruri, 'uri'), (host, 'host')] if not name is None])
      for s in self._hosts:
        if new_set&s:
          for value, id in list(s):
            if id == 'name':
              return value
      return None
    finally:
      self.mutex.release()

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
 