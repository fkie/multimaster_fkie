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
import shutil

from python_qt_binding import QtCore
from python_qt_binding import QtGui

import node_manager_fkie as nm
from common import is_package, package_name
from packages_thread import PackagesThread
from .detailed_msg_box import WarningMessageBox

class LaunchItem(QtGui.QStandardItem):
  '''
  The launch item stored in the launch model. 
  '''
  
  ITEM_TYPE = QtGui.QStandardItem.UserType + 40

  NOT_FOUND = -1
  NOTHING = 0
  RECENT_FILE = 1
  LAUNCH_FILE = 2
  CFG_FILE = 3
  FOLDER = 10
  PACKAGE = 11
  STACK = 12

  def __init__(self, name, path, id, parent=None):
    '''
    Initialize the topic item.
    @param name: the topic name
    @type name: C{str}
    '''
    QtGui.QStandardItem.__init__(self, name)
    self.parent_item = parent
    self.name = name
    self.path = path
    self.package_name = package_name(os.path.dirname(self.path))[0]
    self.id = id
    if self.id == LaunchItem.FOLDER:
      self.setIcon(QtGui.QIcon(":/icons/crystal_clear_folder.png"))
    elif self.id == LaunchItem.PACKAGE:
      self.setIcon(QtGui.QIcon(":/icons/crystal_clear_package.png"))
    elif self.id == LaunchItem.LAUNCH_FILE:
      self.setIcon(QtGui.QIcon(":/icons/crystal_clear_launch_file.png"))
    elif self.id == LaunchItem.RECENT_FILE:
      self.setIcon(QtGui.QIcon(":/icons/crystal_clear_launch_file_recent.png"))
    elif self.id == LaunchItem.STACK:
      self.setIcon(QtGui.QIcon(":/icons/crystal_clear_stack.png"))

#  def __del__(self):
#    print "delete LAUNCH", self.name

  def type(self):
    return LaunchItem.ITEM_TYPE

  def data(self, role):
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
      if self.id == LaunchItem.RECENT_FILE:
        return "%s   [%s]"%(self.name, self.package_name)#.decode(sys.getfilesystemencoding())
      else:
        return "%s"%self.name
    elif role == QtCore.Qt.ToolTipRole:
      # return the tooltip of the item
      result = "%s"%self.path
      if self.id == LaunchItem.RECENT_FILE:
        result = "%s\nPress 'Delete' to remove the entry from the history list"%self.path
      return result
#     elif role == QtCore.Qt.DecorationRole:
#       # return the showed icon
#       pathItem, path, pathId = self.items[index.row()]
#       if self.id > LaunchListModel.NOTHING and self.model().icons.has_key(self.id):
#         return self.model().icons[self.id]
#       return None
    elif role == QtCore.Qt.EditRole:
      return "%s"%self.name
    else:
      # We don't care about anything else, so return default value
      return QtGui.QStandardItem.data(self, role)

  def setData(self, value, role=QtCore.Qt.EditRole):
    if role == QtCore.Qt.EditRole:
      # rename the file or folder
      if self.name != value and self.id in [self.RECENT_FILE, self.LAUNCH_FILE, self.CFG_FILE, self.FOLDER]:
        new_path = os.path.join(os.path.dirname(self.path), value)
        if not os.path.exists(new_path):
          os.rename(self.path, new_path)
          self.name = value
          self.path = new_path
        else:
          WarningMessageBox(QtGui.QMessageBox.Warning, "Path already exists", 
                        "`%s` already exists!"%value, "Complete path: %s"%new_path).exec_()
    return QtGui.QStandardItem.setData(self, value, role)

  @classmethod
  def getItemList(self, name, path, id, root):
    '''
    Creates the list of the items . This list is used for the 
    visualization of topic data as a table row.
    @param name: the topic name
    @type name: C{str}
    @param root: The parent QStandardItem
    @type root: L{PySide.QtGui.QStandardItem}
    @return: the list for the representation as a row
    @rtype: C{[L{LaunchItem} or L{PySide.QtGui.QStandardItem}, ...]}
    '''
    items = []
    item = LaunchItem(name, path, id, parent=root)
    items.append(item)
    return items

  def isLaunchFile(self):
    '''
    @return: C{True} if it is a launch file
    @rtype: C{boolean}
    '''
    return not self.path is None and os.path.isfile(self.path) and self.path.endswith('.launch')

  def isConfigFile(self):
    '''
    @return: C{True} if it is a config file
    @rtype: C{boolean}
    '''
    return self.id == self.CFG_FILE



class LaunchListModel(QtGui.QStandardItemModel):
  '''
  The model to manage the list with launch files.
  '''
  header = [('Name', -1)]
  '''@ivar: the list with columns C{[(name, width), ...]}'''

  def __init__(self):
    '''
    Creates a new list model.
    '''
    QtGui.QStandardItemModel.__init__(self)
    self.setColumnCount(len(LaunchListModel.header))
    self.setHorizontalHeaderLabels([label for label, width in LaunchListModel.header])
    self.pyqt_workaround = dict() # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
    self.items = []
    self.DIR_CACHE = {}
    self.currentPath = None
    self.load_history = self._getLoadHistory()
    self.root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
    self._setNewList(self._moveUp(None))
    self.__packages = {}
    self._fill_packages_thread = PackagesThread()
    self._fill_packages_thread.packages.connect(self._fill_packages)
    self._fill_packages_thread.start()

  def _getRootItems(self):
    result = list(self.load_history)
    result.extend(self.root_paths)
    return result

  def _fill_packages(self, packages):
    self.__packages = packages

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Overloaded methods                    %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def flags(self, index):
    '''
    @param index: parent of the list
    @type index: L{PySide.QtCore.QModelIndex}
    @return: Flag or the requested item
    @rtype: L{PySide.QtCore.Qt.ItemFlag}
    @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
    '''
    if not index.isValid():
      return QtCore.Qt.NoItemFlags
    try:
      item = self.itemFromIndex(index)
      result = QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsDragEnabled
      if item.id in [LaunchItem.RECENT_FILE, LaunchItem.LAUNCH_FILE, LaunchItem.CFG_FILE, LaunchItem.FOLDER]:
        result = result | QtCore.Qt.ItemIsEditable | QtCore.Qt.ItemIsDragEnabled
      return result
    except:
      return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              Drag operation                        %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def mimeTypes(self):
    return ['text/plain']

  def mimeData(self, indexes):
    mimeData = QtCore.QMimeData()
    text = ''
    for index in indexes:
      if index.isValid():
        item = self.itemFromIndex(index)
        prev = '%s\n'%text if text else ''
        text = '%sfile://%s'%(prev, item.path)
    mimeData.setData('text/plain', text)
    return mimeData

  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  #%%%%%%%%%%%%%              External usage                        %%%%%%%%
  #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  def reloadPackages(self):
    '''
    Reloads the cached packag list.
    '''
    if not self._fill_packages_thread.isAlive():
      self._fill_packages_thread = PackagesThread()
      self._fill_packages_thread.packages.connect(self._fill_packages)
      self._fill_packages_thread.start()

  def reloadCurrentPath(self):
    '''
    Reloads the current path.
    '''
    # clear the cache for package names
    try:
      from common import PACKAGE_CACHE
      PACKAGE_CACHE.clear()
      self.DIR_CACHE = {}
    except:
      import traceback
      print traceback.format_exc(2)
    try:
      if self.currentPath is None:
        self._setNewList(self._moveUp(self.currentPath))
      else:
          self._setNewList(self._moveDown(self.currentPath))
    except:
      self._setNewList(self._moveUp(None))

  def expandItem(self, path_item, path, id):
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
    if path_item == '..':
      root_path, items = self._moveUp(os.path.dirname(path))
    elif os.path.isfile(path):
      return path
    elif id == LaunchItem.RECENT_FILE or id == LaunchItem.LAUNCH_FILE:
      raise Exception("Invalid file path: %s", path)
    else:
      root_path, items = self._moveDown(path)
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
    try:
      while len(self.load_history) > nm.settings().launch_history_length:
        self.load_history.pop(0)
    except:
      pass
    self._storeLoadHistory(self.load_history)
#    self.reloadCurrentPath() # todo: keep the item selected in list view after the reload the path

  def removeFromLoadHistory(self, file):
    try:
      self.load_history.remove(file)
    except:
      pass
    self._storeLoadHistory(self.load_history)
#    self.reloadCurrentPath() # todo: keep the item selected in list view after the reload the path

  def show_packages(self, show):
    if show:
    # clear the cache for package names
      try:
        items = []
        for package, path in self.__packages.items():
          items.append((package, path, LaunchItem.PACKAGE))
        self._setNewList((self.currentPath if self.currentPath else '', items))
      except:
        import traceback
        print traceback.format_exc(2)
    else:
      self._setNewList(self._moveUp(self.currentPath))

  def paste_from_clipboard(self):
    '''
    Copy the file or folder to new position...
    '''
    if QtGui.QApplication.clipboard().mimeData().hasText() and self.currentPath:
      text = QtGui.QApplication.clipboard().mimeData().text()
      if text.startswith('file://'):
        path = text.replace('file://', '')
        basename = os.path.basename(text)
        ok = True
        if os.path.exists(os.path.join(self.currentPath, basename)):
          basename, ok = QtGui.QInputDialog.getText(None, 'File exists', 'New name (or override):', QtGui.QLineEdit.Normal, basename)
        if ok and basename:
          if os.path.isdir(path):
            shutil.copytree(path, os.path.join(self.currentPath, basename))
          elif os.path.isfile(path):
            shutil.copy2(path, os.path.join(self.currentPath, basename))
          self.reloadCurrentPath()

  def copy_to_clipboard(self, indexes):
    '''
    Copy the selected path to the clipboard
    '''
    mimeData = QtCore.QMimeData()
    text = ''
    for index in indexes:
      if index.isValid():
        item = self.itemFromIndex(index)
        prev = '%s\n'%text if text else ''
        text = '%sfile://%s'%(prev, item.path)
    mimeData.setData('text/plain', text)
    QtGui.QApplication.clipboard().setMimeData(mimeData)
        
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
    root = self.invisibleRootItem()
    while root.rowCount():
      root.removeRow(0)
    self.pyqt_workaround.clear()
    # add new items
    if not root_path is None:
      self._addPathToList('..', root_path, LaunchItem.NOTHING)
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
    root = self.invisibleRootItem()
    if item is None or path is None or path_id == LaunchItem.NOT_FOUND:
      return False
    if (path_id != LaunchItem.NOT_FOUND):
      # add sorted a new entry
      try:
        for i in range(root.rowCount()):
          launchItem = root.child(i)
          launch_file_cmp = (path_id in [LaunchItem.RECENT_FILE, LaunchItem.LAUNCH_FILE] and item < launchItem.name)
          launch_id_cmp = (launchItem.id > path_id and launchItem.id > LaunchItem.LAUNCH_FILE)
          launch_name_cmp = (launchItem.id == path_id and item < launchItem.name)
          if launch_file_cmp or launch_id_cmp or launch_name_cmp:
            new_item_row = LaunchItem.getItemList(item, path, path_id, root)
            root.insertRow(i, new_item_row)
            self.pyqt_workaround[item] = new_item_row[0] # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
            return True
        new_item_row = LaunchItem.getItemList(item, path, path_id, root)
        root.appendRow(new_item_row)
        self.pyqt_workaround[item] = new_item_row[0]
        return True
      except:
        import traceback
        print traceback.format_exc(2)
      return False

  def _identifyPath(self, path):
    '''
    Determines the id of the given path
    @return: the id represents whether it is a file, package or stack
    @rtype: C{constants of LaunchItem} 
    '''
    if path in self.DIR_CACHE:
      if path in self.load_history:
        return LaunchItem.RECENT_FILE
      return self.DIR_CACHE[path]
    if os.path.basename(path)[0] != '.':
      if path in self.load_history:
        self.DIR_CACHE[path] = LaunchItem.RECENT_FILE
        return LaunchItem.RECENT_FILE
      elif os.path.isfile(path):
        if (path.endswith('.launch')):
          self.DIR_CACHE[path] = LaunchItem.LAUNCH_FILE
          return LaunchItem.LAUNCH_FILE
        else:
          for e in nm.settings().launch_view_file_ext:
            if path.endswith(e):
              self.DIR_CACHE[path] = LaunchItem.CFG_FILE
              return LaunchItem.CFG_FILE
      elif os.path.isdir(path):
        fileList = os.listdir(path)
        if self._containsLaunches(path):
          if 'stack.xml' in fileList:
            self.DIR_CACHE[path] = LaunchItem.STACK
            return LaunchItem.STACK
          elif is_package(fileList):
            self.DIR_CACHE[path] = LaunchItem.PACKAGE
            return LaunchItem.PACKAGE
          else:
            self.DIR_CACHE[path] = LaunchItem.FOLDER
            return LaunchItem.FOLDER
    self.DIR_CACHE[path] = LaunchItem.NOT_FOUND
    return LaunchItem.NOT_FOUND

  def _containsLaunches(self, path):
    '''
    Moves recursively down in the path tree and searches for a launch file. If 
    one is found True will be returned.
    @return: C{True} if the path contains a launch file.
    @rtype: C{boolean}
    '''
    fileList = os.listdir(path)
    for file in fileList:
      file_name, file_extension = os.path.splitext(file)
      if os.path.isfile(os.path.join(path, file)) and (file.endswith('.launch')) or (file_extension in nm.settings().launch_view_file_ext):
        return True
    for file in fileList:
      if os.path.isdir(os.path.join(path, file)):
        if self._containsLaunches(os.path.join(path, file)):
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
        pathItem = '%s (src)'%os.path.basename(os.path.dirname(item))
      pathId = self._identifyPath(item)
      if (pathId != LaunchItem.NOT_FOUND):
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
      item = os.path.normpath(os.path.join(path, file)) if not path is None else file
      pathItem = os.path.basename(item)
      if pathItem == 'src':
        pathItem = '%s (src)'%os.path.basename(os.path.dirname(item))
      pathId = self._identifyPath(item)
      if (pathId != LaunchItem.NOT_FOUND):
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
    historyFile = nm.settings().qsettings(nm.settings().LAUNCH_HISTORY_FILE)
    size = historyFile.beginReadArray("launch_history")
    for i in range(size):
      historyFile.setArrayIndex(i)
      if i >= nm.settings().launch_history_length:
        break
      file = historyFile.value("file")
      if os.path.isfile(file):
        result.append(file)
    historyFile.endArray()
    return result

  def _storeLoadHistory(self, files):
    '''
    Saves the list of recently loaded files to history. The existing history will be replaced!
    @param files: the list with filenames
    @type files: C{[str]}
    '''
    historyFile = nm.settings().qsettings(nm.settings().LAUNCH_HISTORY_FILE)
    historyFile.beginWriteArray("launch_history")
    for i, file in enumerate(files):
      historyFile.setArrayIndex(i)
      historyFile.setValue("file", file)
    historyFile.endArray()
