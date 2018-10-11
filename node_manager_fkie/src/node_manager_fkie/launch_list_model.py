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
from python_qt_binding.QtGui import QIcon, QPixmap, QStandardItem, QStandardItemModel
import os
import shutil
from urlparse import urlparse

import node_manager_fkie as nm

from master_discovery_fkie.common import get_hostname, masteruri_from_master
from node_manager_daemon_fkie.common import equal_uri, get_nmd_url, get_masteruri_from_nmd
from node_manager_daemon_fkie.file_item import FileItem

from .common import package_name, utf8, grpc_join, grpc_split_url, grpc_create_url
from .detailed_msg_box import MessageBox


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
    PACKAGE = 20
    STACK = 21
    FOLDER = 22
    REMOTE_DAEMON = 23

    def __init__(self, path, path_id, mtime, size, name, isnew=False):
        '''
        Initialize the PathItem object with given values. The name of the item is the base name of the path.
        Examples of paths:
            grpc://localhost:12311:                 -> name: @localhost
            grpc://localhost:12311:/absolute/path   -> name: path
        :param str url: the url of node manager daemon
        :param str path: file path
        :param int path_id: identification of the path (folder, package, launch file, ...)
        :param int mtime: modification time
        :param int size: file size
        :param str name: the displayed name
        :param bool isnew: set to True if it a new file
        '''
#        self._url = url
#        self.url_parse_result = urlparse(url)
        self._path = path
        self.id = self._identify_path_on_ext(path) if path_id == self.FILE else path_id
        self._isnew = isnew
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
        self._update_icon()

#  def __del__(self):
#    print "delete LAUNCH", self.name

    def _identify_path_on_ext(self, path, default=10):
        '''
        Determines the id based on file extension.
        :param str path: path
        :return: the id represents whether it is a file, package or stack
        :rtype: constants of PathItem
        '''
        _filename, file_extension = os.path.splitext(path)
        if not file_extension:
            return default
        if file_extension == '.launch':
            return PathItem.LAUNCH_FILE
        elif file_extension == '.nmprofile':
            return PathItem.PROFILE
        elif file_extension in nm.settings().launch_view_file_ext:
            return PathItem.CFG_FILE
        return default

    def _update_icon(self):
        if self.id in [self.NOTHING, self.NOT_FOUND]:
            return
        icon_path = ""
        if self.id == self.FOLDER:
            icon_path = ":/icons/crystal_clear_folder.png"
        elif self.id == self.PACKAGE:
            icon_path = ":/icons/crystal_clear_package.png"
        elif self.id == self.LAUNCH_FILE:
            icon_path = ":/icons/crystal_clear_launch_file.png"
        elif self.id == self.RECENT_FILE:
            icon_path = ":/icons/crystal_clear_launch_file_recent.png"
        elif self.id == self.STACK:
            icon_path = ":/icons/crystal_clear_stack.png"
        elif self.id == self.PROFILE:
            icon_path = ":/icons/crystal_clear_profile.png"
        elif self.id == self.RECENT_PROFILE:
            icon_path = ":/icons/crystal_clear_profile_recent.png"
        elif self.id == self.REMOTE_DAEMON:
            icon_path = ":/icons/stock_connect.png"
        elif self.id == self.ROOT:
            icon_path = ":/icons/back.png"
        if icon_path:
            self.setIcon(QIcon(QPixmap(icon_path).scaled(16, 16)))

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
        elif role == Qt.EditRole:
            return "%s" % self.name
        else:
            # We don't care about anything else, so return default value
            return QStandardItem.data(self, role)

    def setData(self, value, role=Qt.EditRole):
        if role == Qt.EditRole:
            # rename the file or folder
            if (self.name != value or self._isnew) and self.id in [self.RECENT_FILE, self.LAUNCH_FILE, self.RECENT_PROFILE, self.PROFILE, self.CFG_FILE, self.FOLDER]:
                if self.name != value:
                    # some sanity checks
                    if self.model()._exists(value):
                        result = MessageBox.question(self.model().viewobj, "File exists", "File '%s' exists. Override?" % value, buttons=MessageBox.Yes | MessageBox.No)
                        if result == MessageBox.No:
                            return QStandardItem.setData(self, value, role)
                    if self.id not in [self.FOLDER]:
                        _filename, file_extension = os.path.splitext(value)
                        if file_extension not in nm.settings().launch_view_file_ext:
                            result = MessageBox.question(self.model().viewobj, "Unknown extension", "New name has unknown extension '%s'. Rename anyway?" % file_extension, buttons=MessageBox.Yes | MessageBox.No)
                            if result == MessageBox.No:
                                return QStandardItem.setData(self, value, role)
                new_path = os.path.join(os.path.dirname(self.path), value)
                try:
                    # save a new file or rename existing file?
                    content = ''
                    new_id = self._identify_path_on_ext(new_path, self.id)
                    if self._isnew:
                        if new_id in [self.LAUNCH_FILE]:
                            content = ("<launch>\n"
                                       "    <arg name=\"robot_ns\" default=\"my_robot\"/>\n"
                                       "    <group ns=\"$(arg robot_ns)\">\n"
                                       "        <node pkg=\"my_pkg\" type=\"my_node\" name=\"my_name\" >\n"
                                       "            <param name=\"capability_group\" value=\"MY_GROUP\"/>\n"
                                       "        </node>\n"
                                       "    </group>\n"
                                       "</launch>\n")
                        nm.nmd().save_file(new_path, content, 0)
                        self._isnew = False
                    else:
                        nm.nmd().rename(self.path, new_path)
                    # check for new file extension
                    if new_id != self.id:
                        self.id = new_id
                        self._update_icon()
                    if self.name != value and self.id in [self.RECENT_FILE, self.RECENT_PROFILE]:
                        # update in history
                        nm.settings().launch_history_add(new_path, replace=self.path)
                    self.name = value
                    self._path = new_path
                except Exception as err:
                    MessageBox.warning(None, "Rename failed", utf8(err))
        return QStandardItem.setData(self, value, role)

    @classmethod
    def create_row_items(self, path, path_id, mtime, size, name, isnew=False):
        '''
        Creates the list of the items. This list is used for the
        visualization of path data as a table row.
        :param str path: file path combined with the url of node manager daemon
        :param int path_id: identification of the path (folder, package, launch file, ...)
        :param int mtime: modification time
        :param int size: file size
        :param str name: the displayed name
        :param bool isnew: set to True if it a new file
        :return: the list for the representation as a row
        :rtype: C{[L{PathItem} or U{QStandardItem<https://srinikom.github.io/pyside-docs/PySide/QtGui/QStandardItem.html>}, ...]}
        '''
        items = []
        item = PathItem(path, path_id, mtime, size, name, isnew)
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
            return self.path.lower() == item.lower()
        elif not (item is None):
            return self.path.lower() == item.path.lower()
        return False

    def __gt__(self, item):
        '''
        Compares the path of the item.
        '''
        if isinstance(item, str) or isinstance(item, unicode):
            return self.path.lower() > item.lower()
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

    def __init__(self, parent=None, progress_queue=None, viewobj=None):
        '''
        Creates a new list model.
        '''
        QStandardItemModel.__init__(self, parent)
        self.viewobj = viewobj
        self.setColumnCount(len(LaunchListModel.header))
        self.setHorizontalHeaderLabels([label for label, _width in LaunchListModel.header])
        self.pyqt_workaround = dict()  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
        self.items = []
        self._roots = {}
        self._current_path = get_nmd_url()
        self._current_master = masteruri_from_master()
        self._current_master_name = ''
        self.ros_root_paths = {}  # {url: [root pasth(str)]}
        self.ros_root_paths[self._current_path] = [os.path.normpath(p) for p in os.getenv("ROS_PACKAGE_PATH").split(':')]
        self._progress_queue = progress_queue
        nm.nmd().listed_path.connect(self._listed_path)
        nm.nmd().packages_available.connect(self._on_new_packages)
        nm.nmd().error.connect(self._nmd_error)
        nm.nmd().list_path_threaded(self._current_path)
        # self.count = 0

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
        return self._current_master

    @property
    def is_in_root(self):
        '''
        Returns true if the current path is a root path
        :rtype: bool
        '''
        return self._is_root(self._current_path)

    def _is_root(self, grpc_path):
        return grpc_path == get_nmd_url()

    def _is_ros_root(self, grpc_path):
        url, path = grpc_split_url(grpc_path, with_scheme=True)
        if url in self.ros_root_paths and path in self.ros_root_paths[url]:
            return True
        return False

    def set_current_master(self, masteruri, mastername):
        self._current_master = masteruri.rstrip(os.path.sep)
        self._current_master_name = mastername
        if self._is_root(self._current_path):
            nm.nmd().list_path_threaded(self._current_path)
            if equal_uri(self._current_path, masteruri_from_master()):
                self._add_path(get_nmd_url(self._current_master), PathItem.REMOTE_DAEMON, 0, 0, get_hostname(self._current_master_name))

    def is_current_nmd(self, url):
        return equal_uri(get_masteruri_from_nmd(url), get_masteruri_from_nmd(self._current_path))

    def _add_history(self):
        for hitem in nm.settings().launch_history:
            if not hitem.startswith(os.path.sep):
                hitem_uri, _ = grpc_split_url(hitem, with_scheme=True)
                current_uri = get_nmd_url(self._current_path)
                if equal_uri(hitem_uri, current_uri):
                    self._add_path(hitem, PathItem.RECENT_FILE, 0, 0, os.path.basename(hitem))

    def _on_new_packages(self, grpc_url):
        if not self._current_path:
            self.reload_current_path()

    def _listed_path(self, url, path, result):
        if not self.is_current_nmd(url):
            return
        root = self.invisibleRootItem()
        while root.rowCount():
            root.removeRow(0)
        self.pyqt_workaround.clear()
        # test for ROS root paths and add these if it is in ROS_PACKAGE_PATH
        isroot = path in ['', os.path.sep]
        if isroot:
            self.ros_root_paths[url] = []
        # append path items to the list model
        result_list = []
        for path_item in result:
            if isroot and path_item.type in [FileItem.DIR, FileItem.PACKAGE]:
                self.ros_root_paths[url].append(path_item.path)
            item = os.path.normpath(os.path.join(path, path_item.path))
            gpath = grpc_create_url(url, item)
            path_id = PathItem.NOT_FOUND
            if FileItem.FILE == path_item.type:
                print "os.path.splitext(path_item.path)[1]:", os.path.splitext(path_item.path), nm.settings().launch_view_file_ext
                _, ext = os.path.splitext(path_item.path)
                if ext in nm.settings().launch_view_file_ext:
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
        isroot = self._is_root(self._current_path)
        if isroot and not equal_uri(self._current_master, masteruri_from_master()):
            self._add_path(get_nmd_url(self._current_master), PathItem.REMOTE_DAEMON, 0, 0, get_hostname(self._current_master_name))
        self.pathlist_handled.emit(root_path)

    def _nmd_error(self, method, url, path, error):
        if not self.is_current_nmd(url):
            return
#         print "UNAVALABLE:", url
#         if self.count > 0:
#             import time
#             time.sleep(2)
#             nm.nmd().list_path_threaded(self._current_path)
#             return
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
        detail_msg = utf8(error)
        if hasattr(error, 'details'):
            detail_msg = utf8(error.details())
        path_item = PathItem.create_row_items(utf8(error), PathItem.NOTHING, 0, 0, utf8("%s, please start node manager daemon" % detail_msg))
        root.appendRow(path_item)
        self.pyqt_workaround[path_item[0].name] = path_item[0]
        self.error_on_path.emit(grpc_create_url(url, path), error)

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
                text = '%s%s' % (prev, item.path)
        mimeData.setData('text/plain', text.encode('utf-8'))
        return mimeData

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              External usage                        %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def reload_current_path(self, clear_cache=False):
        '''
        Reloads the current path.
        '''
        if clear_cache:
            nm.nmd().list_packages_threaded(self.current_grpc, True)
        self.expand_item(self._current_path, PathItem.FOLDER, clear_cache)

    def expand_item(self, path, path_id, clear_cache=False):
        '''
        Returns for the given item and path the file path if this is a file. Otherwise the
        folder will be expanded and None will be returned.
        :param str path: the real path of the item
        :param int path_id: the id of the path
        :param bool clear_cache: clear cache before reload
        :return: path of the launch file or None
        :rtype: str
        :raise Exception: if no path to given item was found
        '''
        print "expand", path, path_id
        if path_id in [PathItem.NOTHING]:
            return None
        if path_id in [PathItem.LAUNCH_FILE, PathItem.CFG_FILE, PathItem.PROFILE, PathItem.FILE, PathItem.RECENT_FILE, PathItem.LAUNCH_FILE]:
            return path
        root = self.invisibleRootItem()
        while root.rowCount():
            root.removeRow(0)
        self.pyqt_workaround.clear()
        if Qt.ControlModifier & QApplication.keyboardModifiers():
            self._current_path = get_nmd_url()
        else:
            if path_id in [PathItem.ROOT]:
                surl, spath = grpc_split_url(path, with_scheme=True)
                if self._is_root(path) or spath in ['', os.path.sep]:
                    self._current_path = get_nmd_url()
                elif self._is_ros_root(path):
                    self._current_path = surl
                else:
                    dir_path = os.path.dirname(spath)
                    self._current_path = grpc_join(surl, dir_path)
            elif self._current_path != path:
                self._current_path = path
        self._add_path(self._current_path, PathItem.ROOT, 0, 0, 'loading...')
        nm.nmd().list_path_threaded(self._current_path, clear_cache)
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

    def show_packages(self, pattern):
        try:
            root = self.invisibleRootItem()
            while root.rowCount():
                root.removeRow(0)
            self.pyqt_workaround.clear()
            items = []
            currurl = self.current_grpc
            for url, packages in nm.nmd().get_packages().items():
                if url == currurl:
                    for path, name in packages.items():
                        if pattern in name:
                            items.append((grpc_join(url, path), PathItem.PACKAGE, 0, 0, name))
            self._set_new_list(self._current_path, items, add_history=False)
        except Exception:
            import traceback
            print traceback.format_exc(2)

#     def paste_from_clipboard(self):
#         '''
#         Copy the file or folder to new position...
#         '''
#         #TODO
#         if QApplication.clipboard().mimeData().hasText() and self._current_path:
#             text = QApplication.clipboard().mimeData().text()
#             if text.startswith('file://'):
#                 path = text.replace('file://', '')
#                 basename = os.path.basename(text)
#                 ok = True
#                 if os.path.exists(os.path.join(self._current_path, basename)):
#                     basename, ok = QInputDialog.getText(None, 'File exists', 'New name (or override):', QLineEdit.Normal, basename)
#                 if ok and basename:
#                     if os.path.isdir(path):
#                         shutil.copytree(path, os.path.join(self._current_path, basename))
#                     elif os.path.isfile(path):
#                         shutil.copy2(path, os.path.join(self._current_path, basename))
#                     self.reloadcurrent_path()

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
                text = '%s%s' % (prev, item.path)
        mimeData.setData('text/plain', text.encode('utf-8'))
        QApplication.clipboard().setMimeData(mimeData)

    def add_new_launch(self):
        '''
        Inserts the given item in the list model.
        :param str path: the path of the item combined with the url of the node manager daemon
        :param int path_id: the id (constants of PathItem) of the item, which represents whether it is a file, package or stack
        :param int mtime: modification time
        :param int size: file size
        :param str name: the displayed name
        '''
        root = self.invisibleRootItem()
        new_name = 'new.launch'
        cc = 0
        while self._exists(new_name):
            cc += 1
            new_name = 'new_%d.launch' % cc
        # add sorted a new entry
        try:
            path_item = PathItem.create_row_items(grpc_join(self._current_path, new_name), PathItem.LAUNCH_FILE, 0, 0, new_name, isnew=True)
            if root.rowCount() > 1:
                root.insertRow(1, path_item)
            else:
                root.appendRow(path_item)
            self.pyqt_workaround[path_item[0].name] = path_item[0]  # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
            return path_item
        except Exception:
            import traceback
            print traceback.format_exc(2)
        return []

    def _exists(self, name):
        root = self.invisibleRootItem()
        for row in range(root.rowCount()):
            item = root.child(row)
            if item.name == name:
                return True
        return False

    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # %%%%%%%%%%%%%              Functionality                         %%%%%%%%
    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _set_new_list(self, root_path, items, add_history=True):
        '''
        Sets the list to the given path and insert the items. If the root path is not
        empty the additional item '<-' to go back will be inserted.
        :see: L{LaunchListModel._listed_path()}
        :param str root_path: the root directory
        :param items: the list with characterized items
        :type items: C{[(item, path, id)]}
        '''
        # add new items
        _, path = grpc_split_url(root_path)
        self._current_path = root_path
        if not self._is_root(root_path):
            self._add_path(root_path, PathItem.ROOT, 0, 0, '')
        if path in ['', os.path.sep]:
            if add_history:
                self._add_history()
        for path, path_id, mtime, size, name in items:
            self._add_path(path, path_id, mtime, size, name)

    def _add_path(self, path, path_id, mtime, size, name):
        '''
        Inserts the given item in the list model.
        :param str path: the path of the item combined with the url of the node manager daemon
        :param int path_id: the id (constants of PathItem) of the item, which represents whether it is a file, package or stack
        :param int mtime: modification time
        :param int size: file size
        :param str name: the displayed name
        '''
        root = self.invisibleRootItem()
        if path is None or path_id == PathItem.NOT_FOUND:
            return False
        if (path_id != PathItem.NOT_FOUND):
            # add sorted a new entry
            try:
                path_item = PathItem.create_row_items(path, path_id, mtime, size, name)
                for i in range(root.rowCount()):
                    curr_item = root.child(i)
                    launch_file_cmp = path_id in [PathItem.RECENT_FILE, PathItem.LAUNCH_FILE, PathItem.RECENT_PROFILE, PathItem.PROFILE] and curr_item.name < path_item[0].name and curr_item.id not in [PathItem.ROOT]
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
