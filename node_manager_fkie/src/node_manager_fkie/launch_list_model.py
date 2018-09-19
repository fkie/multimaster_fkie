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

from python_qt_binding.QtCore import QMimeData, Qt, Signal
try:
    from python_qt_binding.QtGui import QApplication, QInputDialog, QLineEdit
except Exception:
    from python_qt_binding.QtWidgets import QApplication, QInputDialog, QLineEdit
from python_qt_binding.QtGui import QIcon, QStandardItem, QStandardItemModel
import grpc
import uuid
import os
import shutil
from urlparse import urlparse
import rospy

import node_manager_fkie as nm

from master_discovery_fkie.common import get_hostname, masteruri_from_master
from node_manager_daemon_fkie.common import equal_uri, get_nmd_url
from node_manager_daemon_fkie.file_item import FileItem

from .common import is_package, package_name, utf8, grpc_join, grpc_split_url, grpc_create_url
from .detailed_msg_box import MessageBox
from .packages_thread import PackagesThread


class PathItem(QStandardItem):
    '''
    The launch item stored in the launch model.
    '''

    ITEM_TYPE = QStandardItem.UserType + 40

    NOT_FOUND = -1
    NOTHING = 0
    ROOT = 1
    PROFILE = 5
    RECENT_PROFILE = 6
    RECENT_FILE = 7
    FILE = 10
    LAUNCH_FILE = 11
    CFG_FILE = 12
    FOLDER = 20
    PACKAGE = 21
    STACK = 22
    REMOTE_DAEMON = 23

    def __init__(self, path, path_id, mtime, size, name):
        '''
        Initialize the PathItem object with given values. The name of the item is the base name of the path.
        Examples of paths:
            grpc://localhost:12311:                 -> name: @localhost
            grpc://localhost:12311:/absolute/path   -> name: path
            grpc://localhost:12311:relative/path    -> name: path
            grpc://localhost::with/default/port     -> name: port
            /absolute/local/path                    -> name: path
            relative/local/path                     -> name: path
        :param str url: the url of node manager daemon
        :param str path: file path
        :param int path_id: identification of the path (folder, package, launch file, ...)
        :param int mtime: modification time
        :param int size: file size
        :param str name: the displayed name
        '''
#        self._url = url
#        self.url_parse_result = urlparse(url)
        self._path = path
        self.id = self._identify_path_on_ext(path) if path_id == self.FILE else path_id
        pathname = name
        if pathname == 'src':
            pathname = '%s (src)' % os.path.basename(os.path.dirname(path))
        self._name = pathname
        QStandardItem.__init__(self, self._name)
        self.mtime = mtime
        self.size = size
        if self.id == self.RECENT_FILE or self.id == self.RECENT_PROFILE:
            pname = package_name(path)[0]
            if pname is None:
                pname, _ = grpc_split_url(path, with_scheme=True)
            self.package_name = pname
        if self.id == self.FOLDER:
            self.setIcon(QIcon(":/icons/crystal_clear_folder.png"))
        elif self.id == self.PACKAGE:
            self.setIcon(QIcon(":/icons/crystal_clear_package.png"))
        elif self.id == self.LAUNCH_FILE:
            self.setIcon(QIcon(":/icons/crystal_clear_launch_file.png"))
        elif self.id == self.RECENT_FILE:
            self.setIcon(QIcon(":/icons/crystal_clear_launch_file_recent.png"))
        elif self.id == self.STACK:
            self.setIcon(QIcon(":/icons/crystal_clear_stack.png"))
        elif self.id == self.PROFILE:
            self.setIcon(QIcon(":/icons/crystal_clear_profile.png"))
        elif self.id == self.RECENT_PROFILE:
            self.setIcon(QIcon(":/icons/crystal_clear_profile_recent.png"))
        elif self.id == self.REMOTE_DAEMON:
            self.setIcon(QIcon(":/icons/stock_connect.png"))
        elif self.id == self.ROOT:
            self.setIcon(QIcon(":/icons/back.png"))

#  def __del__(self):
#    print "delete LAUNCH", self.name

    def _identify_path_on_ext(self, path):
        '''
        Determines the id based on file extension.
        :param str path: path
        :return: the id represents whether it is a file, package or stack
        :rtype: constants of PathItem
        '''
        _filename, file_extension = os.path.splitext(path)
        if not file_extension:
            return self.FILE
        if file_extension == '.launch':
            return PathItem.LAUNCH_FILE
        elif file_extension == '.nmprofile':
            return PathItem.PROFILE
        elif file_extension in nm.settings().launch_view_file_ext:
            return PathItem.CFG_FILE
        return self.FILE

    def _update_icon(self):
        if self.id == PathItem.UNKNOWN:
            return
        if self.id == PathItem.FOLDER:
            self.setIcon(QIcon(":/icons/crystal_clear_folder.png"))
        elif self.id == PathItem.PACKAGE:
            self.setIcon(QIcon(":/icons/crystal_clear_package.png"))
        elif self.id == PathItem.LAUNCH_FILE:
            if self.parent_item is None:
                self.setIcon(QIcon(":/icons/crystal_clear_launch_file_recent.png"))
            else:
                self.setIcon(QIcon(":/icons/crystal_clear_launch_file.png"))
        elif self.id == PathItem.STACK:
            self.setIcon(QIcon(":/icons/crystal_clear_stack.png"))
        elif self.id == PathItem.PROFILE:
            if self.parent_item is None:
                self.setIcon(QIcon(":/icons/crystal_clear_profile_recent.png"))
            else:
                self.setIcon(QIcon(":/icons/crystal_clear_profile.png"))
        elif self.id == PathItem.REMOTE_DAEMON:
            self.setIcon(QIcon(":/icons/stock_connect.png"))

    @property
    def name(self):
        '''
        The name of this item.
        :rtype: str
        '''
        return self._name

    @name.setter
    def name(self, new_name):
        '''
        Set the new name of this item and updates the path of the item.
        :param str new_name: new name
        '''
        self._name = new_name
        self.setText(self._name)
        # TODO: update the path

    @property
    def path(self):
        '''
        The path of this item.
        :rtype: str
        '''
        return self._path

#     @property
#     def url(self):
#         '''
#         The url of this item.
#         :rtype: str
#         '''
#         return self._url

    def type(self):
        return PathItem.ITEM_TYPE

    def data(self, role):
        '''
        The view asks us for all sorts of information about our data...
        :param role: the art of the data
        :type role: U{QtCore.Qt.DisplayRole<https://srinikom.github.io/pyside-docs/PySide/QtCore/Qt.html>}
        :see: U{http://www.pyside.org/docs/pyside-1.0.1/PySide/QtCore/Qt.html}
        '''
        if role == Qt.DisplayRole:
            # return the displayed item name
            if self.id == PathItem.RECENT_FILE or self.id == PathItem.RECENT_PROFILE:
                return "%s   [%s]" % (self.name, self.package_name)  # .decode(sys.getfilesystemencoding())
            elif self.id == PathItem.REMOTE_DAEMON:
                return "//%s" % self.name
            else:
                return "%s" % self.name
        elif role == Qt.ToolTipRole:
            # return the tooltip of the item
            result = "%s" % self.path
            if self.id == PathItem.RECENT_FILE or self.id == PathItem.RECENT_PROFILE:
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
        # TODO: fix to use it with nm_daemon
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
    def create_row_items(self, path, path_id, mtime, size, name):
        '''
        Creates the list of the items. This list is used for the
        visualization of path data as a table row.
        :param str path: file path combined with the url of node manager daemon
        :param int path_id: identification of the path (folder, package, launch file, ...)
        :param int mtime: modification time
        :param int size: file size
        :param str name: the displayed name
        :return: the list for the representation as a row
        :rtype: C{[L{PathItem} or U{QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}, ...]}
        '''
        items = []
        item = PathItem(path, path_id, mtime, size, name)
        items.append(item)
        # items.append(item.item_mtime)
        # items.append(item.item_size)
        return items

    def is_launch_file(self):
        '''
        :return: True if it is a launch file
        :rtype: bool
        '''
        return self.path is not None and self.id in [self.LAUNCH_FILE, self.RECENT_FILE] and self.path.endswith('.launch')

    def is_config_file(self):
        '''
        :return: True if it is a config file
        :rtype: bool
        '''
        return self.id == self.CFG_FILE

    def is_profile_file(self):
        '''
        :return: True if it is a node_manager profile file
        :rtype: bool
        '''
        return self.path is not None and self.id in [self.PROFILE, self.RECENT_PROFILE] and self.path.endswith('.nmprofile')

    def __eq__(self, item):
        '''
        Compares the path of the item.
        '''
        if isinstance(item, str) or isinstance(item, unicode):
            return self.lower() == item.lower()
        elif not (item is None):
            return self.path.lower() == item.path.lower()
        return False

    def __gt__(self, item):
        '''
        Compares the path of the item.
        '''
        if isinstance(item, str) or isinstance(item, unicode):
            return self.lower() > item.lower()
        elif not (item is None):
            return self.path.lower() > item.path.lower()
        return False


# ###############################################################################
# #############                LaunchListModel                     ##############
# ###############################################################################


class LaunchListModel(QStandardItemModel):
    '''
    The model to manage the list with launch files.
    '''
    pathlist_handled = Signal(str)
    error_on_path = Signal(str, Exception)

    header = [('Name', -1)]
    '''@ivar: the list with columns C{[(name, width), ...]}'''

    def __init__(self, parent=None, progress_queue=None):
        '''
        Creates a new list model.
        '''
        QStandardItemModel.__init__(self, parent)
        # self._default_url = 'grpc://localhost:12321'
        self.setColumnCount(len(LaunchListModel.header))
        self.setHorizontalHeaderLabels([label for label, _width in LaunchListModel.header])
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
        self.items = []
        self.DIR_CACHE = {}
        self._current_path = ''
        self._current_masteruri = masteruri_from_master()
        self._current_master = masteruri_from_master()
        self._current_master_name = ''
        self._path_tree = []
#        self._current_url = self._default_url
        self.root_paths = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
        # self._setNewList(self._moveUp(None))
        self.__packages = {}
        self._progress_queue = progress_queue
        # self._fill_packages_thread = PackagesThread()
        # self._add_history()
        nm.nmd().listed_path.connect(self._listed_path)
        nm.nmd().packages_available.connect(self._on_new_packages)
        nm.nmd().error.connect(self._nmd_error)
        nm.nmd().list_path_threaded(get_nmd_url())
        self.count = 0

    @property
    def current_path(self):
        '''
        Current path listed in this model. If empty it is a root.
        :rtype: str
        '''
        return self._current_path

    @property
    def current_grpc(self):
        '''
        Current URL of the node manager daemon.
        :rtype: str
        '''
        netloc, _ = grpc_split_url(self._current_path, with_scheme=True)
        return netloc

    @property
    def current_masteruri(self):
        '''
        Current URL of the ROS MASTER.
        :rtype: str
        '''
        return self._current_masteruri

    def set_current_master(self, masteruri, mastername):
        self._current_master = masteruri.rstrip(os.path.sep)
        self._current_master_name = mastername
        if self._current_path == '':
            nm.nmd().list_path_threaded(self._current_path)
#             if masteruri != masteruri_from_master() and masteruri != self._current_masteruri:
#                 self._current_masteruri = masteruri
#                 self._add_path(get_nmd_url(masteruri), PathItem.REMOTE_DAEMON, 0, 0, get_hostname(masteruri))

    def _add_history(self):
        print "HISTORY", nm.settings().launch_history
        for hitem in nm.settings().launch_history:
            hitem_uri, _ = grpc_split_url(hitem, with_scheme=True)
            current_uri = get_nmd_url(self._current_path)
            print "hitem_uri", hitem_uri
            print "current_uri", current_uri
            if equal_uri(hitem_uri, current_uri):
                self._add_path(hitem, PathItem.RECENT_FILE, 0, 0, os.path.basename(hitem))

    def _on_new_packages(self, grpc_url):
        if not self._current_path:
            print "NEW PACAKGES"
            self.reload_current_path()

    def _getRootItems(self):
        result = list(nm.settings().launch_history)
        result.extend(self.root_paths)
        return result

    def _fill_packages(self, packages):
        self.__packages = packages

    def _listed_path(self, url, path, result, store_cache=True):
        root = self.invisibleRootItem()
        while root.rowCount():
            root.removeRow(0)
        self.pyqt_workaround.clear()
#        self._add_history()
        result_list = []
        if (store_cache):
            self.DIR_CACHE[grpc_join(url, path)] = result
        for path_item in result:
            item = os.path.normpath(os.path.join(path, path_item.path))
            gpath = grpc_create_url(url, item)
            path_id = PathItem.NOT_FOUND
            if FileItem.FILE == path_item.type:
                path_id = PathItem.FILE
            elif FileItem.DIR == path_item.type:
                path_id = PathItem.FOLDER
            elif FileItem.SYMLINK == path_item.type:
                pass
            elif FileItem.PACKAGE == path_item.type:
                path_id = PathItem.PACKAGE
            if path_id != PathItem.NOT_FOUND and not os.path.basename(path_item.path).startswith('.'):
                # TODO: create filter for files
                result_list.append((gpath, path_id, path_item.mtime, path_item.size, os.path.basename(path_item.path)))
        root_path = grpc_create_url(url, path)
        self._set_new_list(root_path, result_list)
        print "LLLLLL", self._current_master, masteruri_from_master(), equal_uri(self._current_master, masteruri_from_master())
        if not equal_uri(self._current_master, masteruri_from_master()):
            self._add_path(get_nmd_url(self._current_master), PathItem.REMOTE_DAEMON, 0, 0, get_hostname(self._current_master_name))
        self.pathlist_handled.emit(root_path)

    def _nmd_error(self, method, url, path, error):
        print "UNAVALABLE:", url
        if self.count > 0:
            import time
            time.sleep(2)
            nm.nmd().list_path_threaded(self._current_path)
            return
#         if hasattr(error, "code") and error.code() == grpc.StatusCode.UNAVAILABLE:
#             name = "node_manager_daemon"
#             print "UNAVALABLE:", url
#             self.error_on_path.emit(grpc_create_url(url, path), error)
#             return
#             if self._progress_queue is not None:
#                 rospy.loginfo("Connection problem to %s, try to start <%s>" % (url, name))
#                 self._progress_queue.add2queue(utf8(uuid.uuid4()),
#                                                'start %s' % name,
#                                                nm.starter().runNodeWithoutConfig,
#                                                (get_hostname(url), '%s_fkie' % name, name,
#                                                 nm.nameres().normalize_name(name), [],
#                                                 None, False))
#                 self._progress_queue.start()
#                 self.count = 1
#                 nm.nmd().list_path_threaded(self._current_path)
#             else:
#                 rospy.logwarn("Error while call <%s> on '%s': %s" % (method, url, error))
#                 rospy.logwarn("can't start <%s>, no progress queue defined!" % name)
#         else:
        root = self.invisibleRootItem()
        while root.rowCount():
            root.removeRow(0)
        self.pyqt_workaround.clear()
        self._add_path(self._current_path, PathItem.ROOT, 0, 0, '')
        path_item = PathItem.create_row_items(utf8(error), PathItem.NOTHING, 0, 0, utf8(error.details()))
        root.appendRow(path_item)
        self.pyqt_workaround[path_item[0].name] = path_item[0]
        self.error_on_path.emit(grpc_create_url(url, path), error)

    def get_file_item(self, path):
        '''
        :return: Return an existing ``FileItem`` object corresponding to the given path.
        :rtype: FileItem
        :raise: LookupError if no item for given path was found
        '''
        if not path:
            return self.invisibleRootItem()
        p_res = urlparse(path)
        if p_res.scheme == 'grpc':
            if p_res.hostname is None:
                raise LookupError("can not find FileItem for %s. Invalid hostname in `grpc` scheme!" % path)
            root = self.invisibleRootItem()
            for i in range(root.rowCount()):
                if root.child(i) == "@%s" % p_res.hostname:
                    pass  # TODO
        return self.invisibleRootItem()

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
            if item.id in [PathItem.RECENT_FILE, PathItem.RECENT_PROFILE, PathItem.LAUNCH_FILE, PathItem.CFG_FILE, PathItem.FOLDER, PathItem.PROFILE]:
                result = result | Qt.ItemIsEditable | Qt.ItemIsDragEnabled
            return result
        except Exception:
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
        mimeData.setData('text/plain', text.encode('utf-8'))
        return mimeData

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              External usage                        %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def reloadPackages(self):
        return
        '''
        Reloads the cached packag list.
        '''
        if not self._fill_packages_thread.isAlive():
            self._fill_packages_thread = PackagesThread()
            self._fill_packages_thread.packages.connect(self._fill_packages)
            self._fill_packages_thread.start()

    def reload_current_path(self, clear_cache=False):
        '''
        Reloads the current path.
        '''
        # clear the cache for package names
        if clear_cache:
            nm.nmd().list_packages_threaded(grpc_create_url(self.current_grpc, ''), True)
#         if clear_cache:
#             try:
#                 from .common import PACKAGE_CACHE
#                 PACKAGE_CACHE.clear()
#                 self.DIR_CACHE = {}
#             except Exception:
#                 import traceback
#                 print traceback.format_exc(2)
        self.expand_item(self._current_path, PathItem.FOLDER)
#         try:
#             from .common import PACKAGE_CACHE
#             PACKAGE_CACHE.clear()
#             self.DIR_CACHE = {}
#         except Exception:
#             import traceback
#             print traceback.format_exc(2)
#         try:
#             if self._current_path is None:
#                 self._setNewList(self._moveUp(self._current_path))
#             else:
#                 self._setNewList(self._moveDown(self._current_path))
#         except Exception:
#             self._setNewList(self._moveUp(None))

#    def expand_item(self, path_item, path, item_id, host):
    def expand_item(self, path, path_id):
        '''
        Returns for the given item and path the file path if this is a file. Otherwise the
        folder will be expanded and None will be returned.
        @param path: the real path of the item
        @type path: C{str}
        @return: path of the launch file or None
        @rtype: C{str}
        @raise Exception if no path to given item was found
        '''
        print "expand", path, path_id
#             print "PPP:", path, ", item: ", path_item, host
#             if isinstance(path, FileItem):
#                 print "path", path.path, host
#                 self._nmd_client.list_path_threaded(path.path, host)
#                 root_path = path.path
#                 items = []
#             else:
#                 goto_path = os.path.dirname(path)
#                 key_mod = QApplication.keyboardModifiers()
#                 if key_mod & Qt.ControlModifier:
#                     goto_path = None
#                 root_path, items = self._moveUp(goto_path)
#        elif os.path.isfile(path):
        if path_id in [PathItem.NOTHING]:
            return None
        if path_id in [PathItem.LAUNCH_FILE, PathItem.CFG_FILE, PathItem.PROFILE, PathItem.FILE, PathItem.RECENT_FILE, PathItem.LAUNCH_FILE]:
            return path
#         elif path_id in [PathItem.RECENT_FILE, PathItem.LAUNCH_FILE]:
#             raise Exception("Invalid file path: %s", path)
        root = self.invisibleRootItem()
        while root.rowCount():
            root.removeRow(0)
        self.pyqt_workaround.clear()
#         if path_id == PathItem.REMOTE_DAEMON:
#             print "connect to %s" % path
#             nm.nmd().list_path_threaded(path)
#         else:
        print "list path"
        print "path", path
        print "self._path_tree", self._path_tree
        print "current_path", self._current_path
        if Qt.ControlModifier & QApplication.keyboardModifiers():
            self._path_tree = []
            self._current_path = ''
        else:
            if path_id in [PathItem.ROOT]:
                if self._path_tree:
                    self._current_path = self._path_tree[-1]
                    self._path_tree = self._path_tree[:-1]
            elif self._current_path != path:
                print "ASD", self._current_path, path
                self._path_tree.append(self._current_path)
                self._current_path = path
                # TODO: change current_masteruri
        try:
            curl, cpath = grpc_split_url(self._current_path, with_scheme=True)
            self._listed_path(curl, cpath, self.DIR_CACHE[self._current_path], store_cache=True)
        except KeyError:
            self._add_path(self._current_path, PathItem.ROOT, 0, 0, '')
            nm.nmd().list_path_threaded(self._current_path)
        print "current_path_end", self._current_path
#         else:
#             key_mod = QApplication.keyboardModifiers()
#             onestep = False
#             if key_mod & Qt.ControlModifier:
#                 onestep = True
#             root_path, items = self._moveDown(path, onestep)
#        self._setNewList((root_path, items))
        return None

    def set_path(self, path):
        '''
        Shows the new path in the launch configuration view. Only if the new path
        is in ros package paths
        :param str path: new path
        '''
        # TODO
        toset = path
        if not path.startswith('grpc://'):
            toset = grpc_create_url(self.current_grpc, path)
        self.expand_item(toset, PathItem.FOLDER)
#    if self._is_in_ros_packages(path):
#        self._setNewList(self._moveDown(path))

    def show_packages(self, pattern):
        try:
            root = self.invisibleRootItem()
            while root.rowCount():
                root.removeRow(0)
            self.pyqt_workaround.clear()
            items = []
            for url, packages in nm.nmd().get_packages().items():
                for path, name in packages.items():
                    if pattern in name:
                        print path, url
                        items.append((grpc_join(url, path), PathItem.PACKAGE, 0, 0, name))
            self._set_new_list(self._current_path, items)
        except Exception:
            import traceback
            print traceback.format_exc(2)

    def paste_from_clipboard(self):
        '''
        Copy the file or folder to new position...
        '''
        if QApplication.clipboard().mimeData().hasText() and self._current_path:
            text = QApplication.clipboard().mimeData().text()
            if text.startswith('file://'):
                path = text.replace('file://', '')
                basename = os.path.basename(text)
                ok = True
                if os.path.exists(os.path.join(self._current_path, basename)):
                    basename, ok = QInputDialog.getText(None, 'File exists', 'New name (or override):', QLineEdit.Normal, basename)
                if ok and basename:
                    if os.path.isdir(path):
                        shutil.copytree(path, os.path.join(self._current_path, basename))
                    elif os.path.isfile(path):
                        shutil.copy2(path, os.path.join(self._current_path, basename))
                    self.reloadcurrent_path()

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
        mimeData.setData('text/plain', text.encode('utf-8'))
        QApplication.clipboard().setMimeData(mimeData)

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              Functionality                         %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _set_new_list(self, root_path, items):
        '''
        Sets the list to the given path and insert the items. If the root path is not
        empty the additional item '<-' to go back will be inserted.
        :see: L{LaunchListModel._listed_path()}
        :param str root_path: the root directory
        :param items: the list with characterized items
        :type items: C{[(item, path, id)]}
        '''
        # add new items
#         if root_path:
#             # _addPathToList('..', root_path, PathItem.NOTHING)
#         else:
#             self._addPathToList('localhost', 'localhost:12321', PathItem.REMOTE_DAEMON)
        _, path = grpc_split_url(root_path)
        print "PATHTHTHT", path
        if path and path != os.path.sep:
            self._add_path(self._current_path, PathItem.ROOT, 0, 0, '')
        else:
            self._path_tree = []
            self._add_history()
        for path, path_id, mtime, size, name in items:
            self._add_path(path, path_id, mtime, size, name)

    def _setNewList(self, (root_path, items), url=''):
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
            self._addPathToList('', root_path, PathItem.NOTHING)
        else:
            self._addPathToList('localhost', 'localhost:12321', PathItem.REMOTE_DAEMON)
        for item_name, item_path, item_id in items:
            self._addPathToList(item_name, item_path, item_id)
        self._current_path = root_path

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

    def _add_path(self, path, path_id, mtime, size, name):
        '''
        Inserts the given item in the list model.
        :param str path: the path of the item combined with the url of the node manager daemon
        :param int path_id: the id (constants of PathItem) of the item, which represents whether it is a file, package or stack
        :param int mtime: modification time
        :param int size: file size
        :param str name: the displayed name
        '''
#        print "add path to list", url, path, path_id
        root = self.invisibleRootItem()
        if path is None or path_id == PathItem.NOT_FOUND:
            return False
        if (path_id != PathItem.NOT_FOUND):
            # add sorted a new entry
            try:
                path_item = PathItem.create_row_items(path, path_id, mtime, size, name)
                for i in range(root.rowCount()):
                    curr_item = root.child(i)
                    launch_file_cmp = (path_id in [PathItem.RECENT_FILE, PathItem.LAUNCH_FILE, PathItem.RECENT_PROFILE, PathItem.PROFILE] and curr_item.name < path_item[0].name)
                    launch_id_cmp = (curr_item.id > path_id and curr_item.id > PathItem.LAUNCH_FILE)
                    launch_name_cmp = (curr_item.id == path_id and path_item[0].name < curr_item.name)
                    if launch_file_cmp or launch_id_cmp or launch_name_cmp:
                        root.insertRow(i, path_item)
                        self.pyqt_workaround[path_item[0].name] = path_item[0]  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
                        return True
                root.appendRow(path_item)
                self.pyqt_workaround[path_item[0].name] = path_item[0]
                return True
            except Exception:
                import traceback
                print traceback.format_exc(2)
            return False

    def _identifyPath(self, path):
        '''
        Determines the id of the given path
        @return: the id represents whether it is a file, package or stack
        @rtype: C{constants of PathItem}
        '''
        if path in self.DIR_CACHE:
            if path in nm.settings().launch_history:
                if path.endswith('.nmprofile'):
                    return PathItem.RECENT_PROFILE
                return PathItem.RECENT_FILE
            return self.DIR_CACHE[path]
        if os.path.basename(path)[0] != '.':
            if path in nm.settings().launch_history:
                if path.endswith('.nmprofile'):
                    self.DIR_CACHE[path] = PathItem.RECENT_PROFILE
                    return PathItem.RECENT_PROFILE
                else:
                    self.DIR_CACHE[path] = PathItem.RECENT_FILE
                    return PathItem.RECENT_FILE
            elif os.path.isfile(path):
                if (path.endswith('.launch')):
                    self.DIR_CACHE[path] = PathItem.LAUNCH_FILE
                    return PathItem.LAUNCH_FILE
                elif (path.endswith('.nmprofile')):
                    self.DIR_CACHE[path] = PathItem.PROFILE
                    return PathItem.PROFILE
                else:
                    for e in nm.settings().launch_view_file_ext:
                        if path.endswith(e):
                            self.DIR_CACHE[path] = PathItem.CFG_FILE
                            return PathItem.CFG_FILE
            elif os.path.isdir(path):
                fileList = os.listdir(path)
                if self._containsLaunches(path):
                    if 'stack.xml' in fileList:
                        self.DIR_CACHE[path] = PathItem.STACK
                        return PathItem.STACK
                    elif is_package(fileList):
                        self.DIR_CACHE[path] = PathItem.PACKAGE
                        return PathItem.PACKAGE
                    else:
                        self.DIR_CACHE[path] = PathItem.FOLDER
                        return PathItem.FOLDER
        self.DIR_CACHE[path] = PathItem.NOT_FOUND
        return PathItem.NOT_FOUND

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
            if (pathId != PathItem.NOT_FOUND):
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
            if (pathId != PathItem.NOT_FOUND):
                result_list.append((pathItem, item, pathId))
        if path is not None and len(result_list) == 1 and not os.path.isfile(result_list[0][1]):
            return self._moveUp(os.path.dirname(path))
        else:
            self._current_path = ''
        return path, result_list

