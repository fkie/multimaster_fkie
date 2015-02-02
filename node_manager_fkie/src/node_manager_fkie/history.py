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
from python_qt_binding import QtCore

import rospy

import node_manager_fkie as nm

class History(QtCore.QObject):

  PARAM_CACHE = dict()
  '''
  the cache is used to store and recover the value for last entered parameter in parameter dialog.
  '''

  def __init__(self):
    QtCore.QObject.__init__(self)
    self.PARAM_CACHE = self.loadCache(nm.settings().PARAM_HISTORY_FILE)

  def storeAll(self):
    self.storeCache(nm.settings().PARAM_HISTORY_FILE, self.PARAM_CACHE, nm.settings().param_history_length)

  def cachedParamValues(self, key):
    try:
      return list(self.PARAM_CACHE[key])
    except:
      result = []
      return result

  def addParamCache(self, key, value):
    self._add2Cache(self.PARAM_CACHE, key, value)

  def removeParamCache(self, key, value):
    self._removeFromCache(self.PARAM_CACHE, key, value)

  def loadCache(self, history_file):
    '''
    Loads the content of the given file and return it as cache.
    @param history_file: the name of the history file
    @type history_file: C{str}
    @return: the dictionary with arguments
    @rtype: C{dict(str(name):[str(value), ...], ...)}
    '''
    result = {}
    historyFile = os.path.join(nm.settings().cfg_path, history_file)
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
                elif len(result[key]) <= nm.settings().param_history_length:
                  result[key].append(value)
          line = f.readline()
    return result

  def storeCache(self, history_file, cache, history_len):
    '''
    Stores the cache to a file.
    @param history_file: the name of the history file
    @type history_file: C{str}
    @param cache: the dictionary with values
    @type cache: C{dict}
    @param history_len: the maximal count of value for a key
    @type history_len: C{int}
    '''
    ignored = dict()
    with open(os.path.join(nm.settings().cfg_path, history_file), 'w') as f:
      for key in cache.keys():
        count = 0
        for value in cache[key]:
          if count < history_len:
            try:
              f.write(''.join([key, ':=', value, '\n']))
            except UnicodeEncodeError, e:
              ignored[key] = (value, str(e))
            except:
              import traceback
              rospy.logwarn("Storing history aborted: %s", traceback.format_exc(1))
            count += 1
          else:
            break
    if ignored:
      rospy.logwarn("Error while storing follow keys: %s", str(ignored))

  def _add2Cache(self, cache, key, value):
    uvalue = unicode(value)
    if key and uvalue:
      if not cache.has_key(key):
        cache[key] = [uvalue]
      elif not uvalue in cache[key]:
        cache[key].insert(0, uvalue)
        if len(cache[key]) >= nm.settings().param_history_length:
          cache[key].pop()
      else:
        cache[key].remove(uvalue)
        cache[key].insert(0, uvalue)

  def _removeFromCache(self, cache, key, value):
    uvalue = unicode(value)
    if key and uvalue:
      if cache.has_key(key):
        value_list = cache[key]
        try:
          value_list.remove(uvalue)
        except:
          pass
        if len(value_list) == 0:
          del cache[key]