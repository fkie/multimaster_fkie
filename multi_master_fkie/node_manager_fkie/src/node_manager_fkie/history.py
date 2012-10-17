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
import os
from PySide import QtCore

import node_manager_fkie as nm

class History(QtCore.QObject):
  
  HISTORY_LENGTH = 12
  ''' 
  the history for each required argument to load a launch file.
  ''' 
  ARG_CACHE = dict()
  ARG_HISTORY_FILE = 'arg.history'

  HOSTS_CACHE = dict()
  HOSTS_HISTORY_FILE = 'host.history'
  
  PARAM_CACHE = dict()
  '''
  the cache is used to store and recover the value for last entered parameter in parameter dialog.
  '''
  
  PARAM_HISTORY_FILE = 'param.history'
  
  def __init__(self):
    QtCore.QObject.__init__(self)
    self.ARG_CACHE = self.loadCache(self.ARG_HISTORY_FILE)
    self.PARAM_CACHE = self.loadCache(self.PARAM_HISTORY_FILE)
    self.HOSTS_CACHE = self.loadCache(self.HOSTS_HISTORY_FILE)
    if not self.HOSTS_CACHE.has_key('hosts'):
      self.HOSTS_CACHE['hosts'] = []

  def storeAll(self):
    self.storeCache(self.ARG_HISTORY_FILE, self.ARG_CACHE, self.HISTORY_LENGTH)
    self.storeCache(self.PARAM_HISTORY_FILE, self.PARAM_CACHE, self.HISTORY_LENGTH)
    self.storeCache(self.HOSTS_HISTORY_FILE, self.HOSTS_CACHE, self.HISTORY_LENGTH)
  
  def cachedParamValues(self, key):
    try:
      return list(self.PARAM_CACHE[key])
    except:
      result = []
      return result

  def addParamCache(self, key, value):
    self._add2Cache(self.PARAM_CACHE, key, value)

  def getHostHistory(self):
    '''
    Read the history of the entered robots from the file stored in ROS_HOME path.
    @return: the list with robot names
    @rtype: C{[str]}
    '''
    return list(self.HOSTS_CACHE['hosts'])

  def add2HostHistory(self, host):
    '''
    Adds a host to the history.
    @param host: the host name
    @type host: C{str}
    '''
    self._add2Cache(self.HOSTS_CACHE, 'hosts', host)

  def getArgHistory(self):
    '''
    Returns the arg history cache.
    @return: the dictionary with arguments
    @rtype: C{dict(str(name):[str(value), ...], ...)}
    '''
    return list(self.ARG_CACHE)

  def addToArgHistory(self, key, value):
    '''
    Adds an argument the arg cache.
    @param key: the argument name
    @type key: C{str}
    @param value: the argument value
    @type value: C{str}
    '''
    self._add2Cache(self.ARG_CACHE, key, value)

  def loadCache(self, file):
    '''
    Loads the content of the given file and return it as cache.
    @param file: the name of the history file
    @type file: C{str}
    @return: the dictionary with arguments
    @rtype: C{dict(str(name):[str(value), ...], ...)}
    '''
    result = {}
    historyFile = ''.join([nm.CFG_PATH, file])
    if not os.path.isdir(nm.CFG_PATH):
      os.makedirs(nm.CFG_PATH)
    if os.path.isfile(historyFile):
      with open(historyFile, 'r') as f:
        line = f.readline()
        while line:
          if line:
            line = line.strip()
            if line:
              key, sep, value = line.partition(':=')
              if sep:
                if not key in result.keys():
                  result[key] = [value]
                else:
                  result[key].append(value)
          line = f.readline()
    return result

  def storeCache(self, file, cache, history_len):
    '''
    Stores the cache to a file.
    @param file: the name of the history file
    @type file: C{str}
    @param cache: the dictionary with values
    @type cache: C{dict}
    @param history_len: the maximal count of value for a key
    @type history_len: C{int}
    '''
    if not os.path.isdir(nm.CFG_PATH):
      os.makedirs(nm.CFG_PATH)
    with open(''.join([nm.CFG_PATH, file]), 'w') as f:
      for key in cache.keys():
        count = 0
        for value in cache[key]:
          if count < history_len:
            f.write(''.join([key, ':=', value, '\n']))
            count += 1
          else:
            break

  def _add2Cache(self, cache, key, value):
    uvalue = unicode(value)
    if key and uvalue:
      if not cache.has_key(key):
        cache[key] = [uvalue]
      elif not uvalue in cache[key]:
        cache[key].insert(0, uvalue)
        if len(cache[key]) >= self.HISTORY_LENGTH:
          cache[key].pop()
      else:
        cache[key].remove(uvalue)
        cache[key].insert(0, uvalue)
