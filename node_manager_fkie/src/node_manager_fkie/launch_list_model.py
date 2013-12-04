#          raise Exception(''.join(["No path to file '", str(item), "' found!"]))
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
import sys

from python_qt_binding import QtCore
from python_qt_binding import QtGui

import node_manager_fkie as nm
from common import is_package, package_name

class LaunchListModel(QtCore.QAbstractListModel):
  '''
  The model to manage the files with launch files.
  '''
  
  NOT_FOUND = -1
  NOTHING = 0
  RECENT_FILE = 1
  LAUNCH_FILE = 2
  FOLDER = 3
  PACKAGE = 4
  STACK = 5

  RECENT_LENGTH = 5

  def __init__(self):
    '''
    Creates a new list model. Loads the required icons.
    '''
    QtCore.QAbstractListModel.__init__(self)
    self.icons = {LaunchListModel.FOLDER : QtGui.QIcon(":/icons/crystal_clear_folder.png"),
                  LaunchListModel.STACK : QtGui.QIcon(":/icons/crystal_clear_stack.png"),
                  LaunchListModel.PACKAGE : QtGui.QIcon(":/icons/crystal_clear_package.png"),
                  LaunchListModel.LAUNCH_FILE : QtGui.QIcon(":/icons/crystal_clear_launch_file.png"),
                  LaunchListModel.RECENT_FILE : QtGui.QIcon(":/icons/crystal_clear_launch_file_recent.png")}
    self.items = []
    self.currentPath = None
    self.load_history = self._getLoadHistory()
    self.root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
    self._setNewList(self._moveUp(None))

  def _getRootItems(self):
    result = list(self.load_history)
    result.extend(self.root_paths)
    return result

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Overloaded methods                    %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def rowCount(self, parent=QtCore.QModelIndex()):
    '''
    Tell the view how many rows we have present in our data.
    @param parent: parent of the list
    @type parent: L{QtCore.QModelIndex}
    '''
    return len(self.items)

  def data(self, index, role=QtCore.Qt.DisplayRole):
    '''
    The view asks us for all sorts of information about our data...
    @param index: parent of the list
    @type index: L{QtCore.QModelIndex}
    @param role: the art of the data
    @type role: L{QtCore.Qt.DisplayRole}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    if role == QtCore.Qt.DisplayRole:
      # return the displayed item name
      pathItem, path, pathId = self.items[index.row()]
      if pathId == LaunchListModel.RECENT_FILE:
        return ''.join([pathItem, '   [', str(package_name(os.path.dirname(path))[0]), ']']).decode(sys.getfilesystemencoding())
      else:
        return pathItem
    elif role == QtCore.Qt.ToolTipRole:
      # return the tooltip of the item
      pathItem, result, pathId = self.items[index.row()]
      if pathId == LaunchListModel.RECENT_FILE:
        result = '\n'.join([result, "Press 'Delete' to remove the entry from the history list"])
      return result.decode(sys.getfilesystemencoding())
    elif role == QtCore.Qt.DecorationRole:
      # return the showed icon
      pathItem, path, pathId = self.items[index.row()]
      if pathId > LaunchListModel.NOTHING and self.icons.has_key(pathId):
        return self.icons[pathId]
      return None
    else:
     # We don't care about anything else, so return None
     return None

  def flags(self, index):
    '''
    Make the only selectable
    @param index: parent of the list
    @type index: L{QtCore.QModelIndex}
    @return: Flag or the requestet item
    @rtype: L{PySide.QtCore.Qt.ItemFlag}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    return QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled


  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              External usage                        %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def reloadCurrentPath(self):
    '''
    Reloads the current path.
    '''
    # clear the cache for package names
    try:
      from common import PACKAGE_CACHE
      PACKAGE_CACHE.clear()
    except:
      import traceback
      print traceback.format_exc()
    if self.currentPath is None:
      self._setNewList(self._moveUp(self.currentPath))
    else:
      self._setNewList(self._moveDown(self.currentPath))

  def isLaunchFile(self, row):
    '''
    Tests for the given row whether it is a launch file or not.
    @return: C{True} if it is a launch file
    @rtype: C{boolean}
    '''
    if row >= 0 and row < len(self.items):
      pathItem, path, pathId = self.items[row]
      return not path is None and os.path.isfile(path) and path.endswith('.launch')
    return False

  def expandItem(self, path_item, path):
    '''
    Returns for the given item and path the file path if this is a file. Otherwise the 
    folder will be expanded and None will be returned.
    @param path_item: the list item
    @type path_item: C{str}
    @param path: the real path of the item
    @type path: C{str}
    @return: path of the launch file or None
    @rtype: C{str
    @raise Exception if no path to given item was found
    '''
    for pathItem, p, id in self.items:
      if path_item == pathItem and p == path:
        if path_item == '..':
          root_path, items = self._moveUp(os.path.dirname(p))
        elif os.path.isfile(p):
          return p
        elif id == LaunchListModel.RECENT_FILE or id == LaunchListModel.LAUNCH_FILE:
          raise Exception(''.join(["Invalid file path: ", str(path)]))
        else:
          root_path, items = self._moveDown(p)
        self._setNewList((root_path, items))
    return None


  def setPath(self, path):
    '''
    Shows the new path in the launch configuration view. Only if the new path
    is in ros package paths
    @param path: new path
    @type path: C{str}
    '''
#    if self._is_in_ros_packages(path):
    self._setNewList(self._moveDown(path))

  def add2LoadHistory(self, file):
    try:
      self.load_history.remove(file)
    except:
      pass
    self.load_history.append(file)
    if len(self.load_history) > self.RECENT_LENGTH:
      self.load_history.pop(0)
    self._storeLoadHistory(self.load_history)
    self.reloadCurrentPath()

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Functionality                         %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def _setNewList(self, (root_path, items)):
    '''
    Sets the list to the given path and insert the items. If the root path is not
    None the additional item '..' to go up will be inserted. The items parameter 
    is a tupel with three values (the displayed name, the path of the item, the id
    of the item). 
    @see: L{LaunchListModel._addPathToList()}
    @param root_path: the root directory
    @type root_path: C{str}
    @param items: the list with characterized items
    @type items: C{[(item, path, id)]}
    '''
    self.beginRemoveRows(QtCore.QModelIndex(), 0, len(self.items))
    del self.items[:]
    self.endRemoveRows()
    # add new items
    if not root_path is None:
      self._addPathToList('..', root_path, LaunchListModel.NOTHING)
    for item_name, item_path, item_id in items:
      self._addPathToList(item_name, item_path, item_id)
    self.currentPath = root_path

  def _is_in_ros_packages(self, path):
    '''
    Test whether the given path is in ROS_PACKAGE_PATH.
    @return: C{True}, if the path is in the ROS_PACKAGE_PATH
    @rtype: C{boolean}
    '''
    #TODO fix for paths with symbolic links
    for p in self.root_paths:
      if path.startswith(p):
        return True
    return False

  def _addPathToList(self, item, path, path_id):
    '''
    Inserts the given item in the list model.
    @param item: the displayed name
    @type item: C{str} 
    @param path: the path of the item
    @type path: C{str} 
    @param path_id: the id of the item, which represents whether it is a file, package or stack.
    @type path_id: C{constants of LaunchListModel} 
    '''
    if item is None or path is None or path_id == LaunchListModel.NOT_FOUND:
      return False
    if (path_id != LaunchListModel.NOT_FOUND):
      # add sorted a new entry
      for index, (i, p, id) in enumerate(self.items):
        launch_file_cmp = (path_id == LaunchListModel.RECENT_FILE and id == LaunchListModel.LAUNCH_FILE and i > item)
        if launch_file_cmp or (id == path_id and i > item): 
          self.beginInsertRows(QtCore.QModelIndex(), index, index)
          self.items.insert(index, (item, path, path_id))
          self.endInsertRows()
          return True
      self.beginInsertRows(QtCore.QModelIndex(), len(self.items), len(self.items))
      self.items.append((item, path, path_id))
      self.endInsertRows()
      return True
    return False

  def _identifyPath(self, path):
    '''
    Determines the id of the given path
    @return: the id represents whether it is a file, package or stack
    @rtype: C{constants of LaunchListModel} 
    '''
    if os.path.basename(path)[0] != '.':
      if path in self.load_history:
        return LaunchListModel.RECENT_FILE
      elif os.path.isfile(path):
        if (path.endswith('.launch')):
          return LaunchListModel.LAUNCH_FILE
      elif os.path.isdir(path):
        fileList = os.listdir(path)
        if self._containsLaunches(path):
          if 'stack.xml' in fileList:
            return LaunchListModel.STACK
          elif is_package(fileList):
            return LaunchListModel.PACKAGE
          else:
            return LaunchListModel.FOLDER
    return LaunchListModel.NOT_FOUND

  def _containsLaunches(self, path):
    '''
    Moves recursively down in the path tree and searches for a launch file. If 
    one is found True will be returned.
    @return: C{True} if the path contains a launch file.
    @rtype: C{boolean}
    '''
    fileList = os.listdir(path)
    for file in fileList:
      if os.path.isfile(''.join([path, '/', file])) and file.endswith('.launch'):
        return True
    for file in fileList:
      if os.path.isdir(''.join([path, '/', file])):
        if self._containsLaunches(''.join([path, '/', file])):
          return True
    return False


  def _moveDown(self, path):
    '''
    Moves recursively down in the path tree until the current path contains a 
    launch file.
    @return: tupel of (root_path, items)
    @rtype: C{tupel of (root_path, items)} 
    @see: L{LaunchListModel._setNewList()}
    '''
    result_list = []
    dirlist = os.listdir(path)
    for file in dirlist:
      item = os.path.normpath(''.join([path, '/', file]))
      pathItem = os.path.basename(item)
      if pathItem == 'src':
        pathItem = ''.join([os.path.basename(os.path.dirname(item)), ' (src)'])
      pathId = self._identifyPath(item)
      if (pathId != LaunchListModel.NOT_FOUND):
        result_list.append((pathItem, item, pathId))
    if len(result_list) == 1 and not os.path.isfile(result_list[0][1]):
      return self._moveDown(result_list[0][1])
    return path, result_list

  def _moveUp(self, path):
    '''
    Moves recursively up in the path tree until the current path contains a 
    launch file or the root path defined by ROS_PACKAGES_PATH is reached.
    @return: tupel of (root_path, items)
    @rtype: C{tupel of (root_path, items)} 
    @see: L{LaunchListModel._setNewList()}
    '''
    result_list = []
    if path is None or not self._is_in_ros_packages(path):
      dirlist = self._getRootItems()
      path = None
    else:
      dirlist = os.listdir(path)
    for file in dirlist:
      item = os.path.normpath(''.join([path, '/', file])) if not path is None else file
      pathItem = os.path.basename(item)
      if pathItem == 'src':
        pathItem = ''.join([os.path.basename(os.path.dirname(item)), ' (src)'])
      pathId = self._identifyPath(item)
      if (pathId != LaunchListModel.NOT_FOUND):
        result_list.append((pathItem, item, pathId))
    if not path is None and len(result_list) == 1 and not os.path.isfile(result_list[0][1]):
      return self._moveUp(os.path.dirname(path))
    else:
      self.currentPath = None
    return path, result_list

  def _getLoadHistory(self):
    '''
    Read the history of the recently loaded files from the file stored in ROS_HOME path.
    @return: the list with file names
    @rtype: C{[str]}
    '''
    result = list()
    historyFile = ''.join([nm.CFG_PATH, 'launch.history'])
    if os.path.isfile(historyFile):
      with open(historyFile, 'r') as f:
        line = f.readline()
        while line:
          if line:
            if os.path.isfile(line.strip()):
              result.append(line.strip())
          line = f.readline()
      f.closed
    return result

  def _storeLoadHistory(self, files):
    '''
    Saves the list of recently loaded files to history. The existing history will be replaced!
    @param files: the list with filenames
    @type files: C{[str]}
    '''
    if not os.path.isdir(nm.CFG_PATH):
      os.makedirs(nm.CFG_PATH)
    with open(''.join([nm.CFG_PATH, 'launch.history']), 'w') as f:
      for files in files:
        f.write(''.join([files, '\n']))
    f.closed
