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

from python_qt_binding.QtCore import QMimeData, Qt
try:
    from python_qt_binding.QtGui import QApplication, QInputDialog, QLineEdit
except:
    from python_qt_binding.QtWidgets import QApplication, QInputDialog, QLineEdit
from python_qt_binding.QtGui import QIcon, QStandardItem, QStandardItemModel
import os
import shutil

import node_manager_fkie as nm

from .common import is_package, package_name, utf8
from .detailed_msg_box import MessageBox
from .packages_thread import PackagesThread


class LaunchItem(QStandardItem):
    '''
    The launch item stored in the launch model.
    '''

    ITEM_TYPE = QStandardItem.UserType + 40

    NOT_FOUND = -1
    NOTHING = 0
    PROFILE = 5
    RECENT_PROFILE = 6
    RECENT_FILE = 10
    LAUNCH_FILE = 11
    CFG_FILE = 12
    FOLDER = 20
    PACKAGE = 21
    STACK = 22

    def __init__(self, name, path, lid, parent=None):
        '''
        Initialize the topic item.
        @param name: the topic name
        @type name: C{str}
        '''
        QStandardItem.__init__(self, name)
        self.parent_item = parent
        self.name = name
        self.path = path
        self.package_name = package_name(os.path.dirname(self.path))[0]
        self.id = lid
        if self.id == LaunchItem.FOLDER:
            self.setIcon(QIcon(":/icons/crystal_clear_folder.png"))
        elif self.id == LaunchItem.PACKAGE:
            self.setIcon(QIcon(":/icons/crystal_clear_package.png"))
        elif self.id == LaunchItem.LAUNCH_FILE:
            self.setIcon(QIcon(":/icons/crystal_clear_launch_file.png"))
        elif self.id == LaunchItem.RECENT_FILE:
            self.setIcon(QIcon(":/icons/crystal_clear_launch_file_recent.png"))
        elif self.id == LaunchItem.STACK:
            self.setIcon(QIcon(":/icons/crystal_clear_stack.png"))
        elif self.id == LaunchItem.PROFILE:
            self.setIcon(QIcon(":/icons/crystal_clear_profile.png"))
        elif self.id == LaunchItem.RECENT_PROFILE:
            self.setIcon(QIcon(":/icons/crystal_clear_profile_recent.png"))

#  def __del__(self):
#    print "delete LAUNCH", self.name

    def type(self):
        return LaunchItem.ITEM_TYPE

    def data(self, role):
        '''
        The view asks us for all sorts of information about our data...
        @param role: the art of the data
        @type role: U{QtCore.Qt.DisplayRole<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if role == Qt.DisplayRole:
            # return the displayed item name
            if self.id == LaunchItem.RECENT_FILE or self.id == LaunchItem.RECENT_PROFILE:
                return "%s   [%s]" % (self.name, self.package_name)  # .decode(sys.getfilesystemencoding())
            else:
                return "%s" % self.name
        elif role == Qt.ToolTipRole:
            # return the tooltip of the item
            result = "%s" % self.path
            if self.id == LaunchItem.RECENT_FILE or self.id == LaunchItem.RECENT_PROFILE:
                result = "%s\nPress 'Delete' to remove the entry from the history list" % self.path
            return result
#     elif role == Qt.DecorationRole:
#       # return the showed icon
#       pathItem, path, pathId = self.items[index.row()]
#       if self.id > LaunchListModel.NOTHING and self.model().icons.has_key(self.id):
#         return self.model().icons[self.id]
#       return None
        elif role == Qt.EditRole:
            return "%s" % self.name
        else:
            # We don't care about anything else, so return default value
            return QStandardItem.data(self, role)

    def setData(self, value, role=Qt.EditRole):
        if role == Qt.EditRole:
            # rename the file or folder
            if self.name != value and self.id in [self.RECENT_FILE, self.LAUNCH_FILE, self.RECENT_PROFILE, self.PROFILE, self.CFG_FILE, self.FOLDER]:
                new_path = os.path.join(os.path.dirname(self.path), value)
                if not os.path.exists(new_path):
                    os.rename(self.path, new_path)
                    if self.name != value and self.id in [self.RECENT_FILE, self.RECENT_PROFILE]:
                        # update in history
                        nm.settings().launch_history_add(new_path, replace=self.path)
                    self.name = value
                    self.path = new_path
                else:
                    MessageBox.warning(self, "Path already exists",
                                       "`%s` already exists!" % value, "Complete path: %s" % new_path)
        return QStandardItem.setData(self, value, role)

    @classmethod
    def getItemList(self, name, path, item_id, root):
        '''
        Creates the list of the items . This list is used for the
        visualization of topic data as a table row.
        @param name: the topic name
        @type name: C{str}
        @param root: The parent QStandardItem
        @type root: U{QtGui.QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}
        @return: the list for the representation as a row
        @rtype: C{[L{LaunchItem} or U{QtGui.QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}, ...]}
        '''
        items = []
        item = LaunchItem(name, path, item_id, parent=root)
        items.append(item)
        return items

    def isLaunchFile(self):
        '''
        @return: C{True} if it is a launch file
        @rtype: C{boolean}
        '''
        return self.path is not None and os.path.isfile(self.path) and self.path.endswith('.launch')

    def isConfigFile(self):
        '''
        @return: C{True} if it is a config file
        @rtype: C{boolean}
        '''
        return self.id == self.CFG_FILE

    def isProfileFile(self):
        '''
        @return: C{True} if it is a node_manager profile file
        @rtype: C{boolean}
        '''
        return self.path is not None and os.path.isfile(self.path) and self.path.endswith('.nmprofile')


class LaunchListModel(QStandardItemModel):
    '''
    The model to manage the list with launch files.
    '''
    header = [('Name', -1)]
    '''@ivar: the list with columns C{[(name, width), ...]}'''

    def __init__(self):
        '''
        Creates a new list model.
        '''
        QStandardItemModel.__init__(self)
        self.setColumnCount(len(LaunchListModel.header))
        self.setHorizontalHeaderLabels([label for label, _width in LaunchListModel.header])
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
        self.items = []
        self.DIR_CACHE = {}
        self.currentPath = None
        self.root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
        self._setNewList(self._moveUp(None))
        self.__packages = {}
        self._fill_packages_thread = PackagesThread()

    def _getRootItems(self):
        result = list(nm.settings().launch_history)
        result.extend(self.root_paths)
        return result

    def _fill_packages(self, packages):
        self.__packages = packages

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              Overloaded methods                    %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def flags(self, index):
        '''
        @param index: parent of the list
        @type index: U{QtCore.QModelIndex<https://srinikom.github.io/pyside-docs/PySide/QtCore/QModelIndex.html>}
        @return: Flag or the requested item
        @rtype: U{QtCore.Qt.ItemFlag<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        @see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if not index.isValid():
            return Qt.NoItemFlags
        try:
            item = self.itemFromIndex(index)
            result = Qt.ItemIsSelectable | Qt.ItemIsEnabled | Qt.ItemIsDragEnabled
            if item.id in [LaunchItem.RECENT_FILE, LaunchItem.RECENT_PROFILE, LaunchItem.LAUNCH_FILE, LaunchItem.CFG_FILE, LaunchItem.FOLDER, LaunchItem.PROFILE]:
                result = result | Qt.ItemIsEditable | Qt.ItemIsDragEnabled
            return result
        except:
            return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              Drag operation                        %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def mimeTypes(self):
        return ['text/plain']

    def mimeData(self, indexes):
        mimeData = QMimeData()
        text = ''
        for index in indexes:
            if index.isValid():
                item = self.itemFromIndex(index)
                prev = '%s\n' % text if text else ''
                text = '%sfile://%s' % (prev, item.path)
        mimeData.setData('text/plain', text)
        return mimeData

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              External usage                        %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
            from .common import PACKAGE_CACHE
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

    def expandItem(self, path_item, path, item_id):
        '''
        Returns for the given item and path the file path if this is a file. Otherwise the
        folder will be expanded and None will be returned.
        @param path_item: the list item
        @type path_item: C{str}
        @param path: the real path of the item
        @type path: C{str}
        @return: path of the launch file or None
        @rtype: C{str}
        @raise Exception if no path to given item was found
        '''
        if path_item == '..':
            goto_path = os.path.dirname(path)
            key_mod = QApplication.keyboardModifiers()
            if key_mod & Qt.ControlModifier:
                goto_path = None
            root_path, items = self._moveUp(goto_path)
        elif os.path.isfile(path):
            return path
        elif item_id == LaunchItem.RECENT_FILE or item_id == LaunchItem.LAUNCH_FILE:
            raise Exception("Invalid file path: %s", path)
        else:
            key_mod = QApplication.keyboardModifiers()
            onestep = False
            if key_mod & Qt.ControlModifier:
                onestep = True
            root_path, items = self._moveDown(path, onestep)
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
        if QApplication.clipboard().mimeData().hasText() and self.currentPath:
            text = QApplication.clipboard().mimeData().text()
            if text.startswith('file://'):
                path = text.replace('file://', '')
                basename = os.path.basename(text)
                ok = True
                if os.path.exists(os.path.join(self.currentPath, basename)):
                    basename, ok = QInputDialog.getText(None, 'File exists', 'New name (or override):', QLineEdit.Normal, basename)
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
        mimeData = QMimeData()
        text = ''
        for index in indexes:
            if index.isValid():
                item = self.itemFromIndex(index)
                prev = '%s\n' % text if text else ''
                text = '%sfile://%s' % (prev, item.path)
        mimeData.setData('text/plain', utf8(text))
        QApplication.clipboard().setMimeData(mimeData)

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              Functionality                         %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
        if root_path is not None:
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
        # TODO fix for paths with symbolic links
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
                    launch_file_cmp = (path_id in [LaunchItem.RECENT_FILE, LaunchItem.LAUNCH_FILE, LaunchItem.RECENT_PROFILE, LaunchItem.PROFILE] and item < launchItem.name)
                    launch_id_cmp = (launchItem.id > path_id and launchItem.id > LaunchItem.LAUNCH_FILE)
                    launch_name_cmp = (launchItem.id == path_id and item < launchItem.name)
                    if launch_file_cmp or launch_id_cmp or launch_name_cmp:
                        new_item_row = LaunchItem.getItemList(item, path, path_id, root)
                        root.insertRow(i, new_item_row)
                        self.pyqt_workaround[item] = new_item_row[0]  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
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
            if path in nm.settings().launch_history:
                if path.endswith('.nmprofile'):
                    return LaunchItem.RECENT_PROFILE
                return LaunchItem.RECENT_FILE
            return self.DIR_CACHE[path]
        if os.path.basename(path)[0] != '.':
            if path in nm.settings().launch_history:
                if path.endswith('.nmprofile'):
                    self.DIR_CACHE[path] = LaunchItem.RECENT_PROFILE
                    return LaunchItem.RECENT_PROFILE
                else:
                    self.DIR_CACHE[path] = LaunchItem.RECENT_FILE
                    return LaunchItem.RECENT_FILE
            elif os.path.isfile(path):
                if (path.endswith('.launch')):
                    self.DIR_CACHE[path] = LaunchItem.LAUNCH_FILE
                    return LaunchItem.LAUNCH_FILE
                elif (path.endswith('.nmprofile')):
                    self.DIR_CACHE[path] = LaunchItem.PROFILE
                    return LaunchItem.PROFILE
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
        for cfile in fileList:
            _, file_extension = os.path.splitext(cfile)
            if os.path.isfile(os.path.join(path, cfile)) and (cfile.endswith('.launch')) or (file_extension in nm.settings().launch_view_file_ext):
                return True
        for cfile in fileList:
            if os.path.isdir(os.path.join(path, cfile)):
                if self._containsLaunches(os.path.join(path, cfile)):
                    return True
        return False

    def _moveDown(self, path, onestep=False):
        '''
        Moves recursively down in the path tree until the current path contains a
        launch file.
        @return: tupel of (root_path, items)
        @rtype: C{tupel of (root_path, items)}
        @see: L{LaunchListModel._setNewList()}
        '''
        result_list = []
        dirlist = os.listdir(path)
        for cfile in dirlist:
            item = os.path.normpath(''.join([path, '/', cfile]))
            pathItem = os.path.basename(item)
            if pathItem == 'src':
                pathItem = '%s (src)' % os.path.basename(os.path.dirname(item))
            pathId = self._identifyPath(item)
            if (pathId != LaunchItem.NOT_FOUND):
                result_list.append((pathItem, item, pathId))
        if len(result_list) == 1 and not os.path.isfile(result_list[0][1]):
            if not onestep:
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
        for dfile in dirlist:
            item = os.path.normpath(os.path.join(path, dfile)) if path is not None else dfile
            pathItem = os.path.basename(item)
            if pathItem == 'src':
                pathItem = '%s (src)' % os.path.basename(os.path.dirname(item))
            pathId = self._identifyPath(item)
            if (pathId != LaunchItem.NOT_FOUND):
                result_list.append((pathItem, item, pathId))
        if path is not None and len(result_list) == 1 and not os.path.isfile(result_list[0][1]):
            return self._moveUp(os.path.dirname(path))
        else:
            self.currentPath = None
        return path, result_list
