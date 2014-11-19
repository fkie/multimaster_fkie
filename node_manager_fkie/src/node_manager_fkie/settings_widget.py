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

from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding import loadUi

import node_manager_fkie as nm

from .settings_model import SettingsModel, SettingsValueItem


class SettingsWidget(QtGui.QDockWidget):
  '''
  Settings widget to handle the settings changes. The changes will direct change
  the settings of the GUI.
  '''

  def __init__(self, parent=None):
    '''
    Creates the window, connects the signals and init the class.
    '''
    QtGui.QDockWidget.__init__(self, parent)
    # load the UI file
    settings_dock_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'SettingsDockWidget.ui')
    loadUi(settings_dock_file, self)
    # initialize the settings view model
    self.settings_model = SettingsModel()
    self.settings_proxyModel = QtGui.QSortFilterProxyModel(self)
    self.settings_proxyModel.setSourceModel(self.settings_model)
    self.settingsTreeView.setModel(self.settings_proxyModel)
    self.settingsTreeView.setAlternatingRowColors(True)
    for i, (_, width) in enumerate(SettingsModel.header):
      self.settingsTreeView.setColumnWidth(i, width)
    self.item_delegate = ItemDelegate()
    self.item_delegate.settings_path_changed_signal.connect(self.reload_settings)
    self.settingsTreeView.setItemDelegateForColumn(1, self.item_delegate)
    self.reload_settings()

  def reload_settings(self):
    '''
    Load the current settings data into the model. The settings itself will not
    be loaded.
    '''
    settings = {'Default user:' : ({
                                   'value' : nm.settings().default_user,
                                   'settings' : nm.settings(),
                                   'attrname' : 'default_user',
                                   'value_default' : nm.settings().USER_DEFAULT,
                                   'tooltip'  : '<p>The user used for ssh connection to remote hosts</p>'
                                   },),
                'Launch history length:' : ({
                                   'value' : nm.settings().launch_history_length,
                                   'settings' : nm.settings(),
                                   'attrname' : 'launch_history_length',
                                   'value_default' : nm.settings().LAUNCH_HISTORY_LENGTH,
                                   'value_min' : 0,
                                   'value_max' : 25,
                                   'tooltip'  : '<p>The count of recent '
                                      'loaded launch files displayed in the root '
                                      'of the <span style=" font-weight:600;">launch '
                                      'files</span> view.</p>'
                                   },),
                'Param history length:' : ({
                                   'value' : nm.settings().param_history_length,
                                   'settings' : nm.settings(),
                                   'attrname' : 'param_history_length',
                                   'value_default' : nm.settings().PARAM_HISTORY_LENGTH,
                                   'value_min' : 0,
                                   'value_max' : 25,
                                   'tooltip'  : '<p>The count of parameters stored which '
                                      'are entered in a parameter dialog (Launch file arguments, '
                                      'paramter server, publishing to a topic, service call)</p>'
                                   },),

                'Settings path:' : ({
                                   'value' : nm.settings().cfg_path,
                                   'settings' : nm.settings(),
                                   'attrname' : 'cfg_path',
                                   'edit_type' : SettingsValueItem.EDIT_TYPE_FOLDER,
                                   'value_default' : nm.settings().CFG_PATH,
                                   'tooltip'  : ''
                                   },),
                'Robot icon path:' : ({
                                   'value' : nm.settings().robots_path,
                                   'settings' : nm.settings(),
                                   'attrname' : 'robots_path',
                                   'edit_type' : SettingsValueItem.EDIT_TYPE_FOLDER,
                                   'value_default' : nm.settings().ROBOTS_DIR,
                                   'tooltip'  : '<p>The path to the folder with robot images '
                                      '(<span style=" font-weight:600;">.png</span>).'
                                      'The images with robot name will be displayed in the '
                                      'info bar.</p>'
                                   },),
                'Show files extensions:' : ({
                                   'value' : ', '.join(nm.settings().launch_view_file_ext),
                                   'settings' : nm.settings(),
                                   'attrname' : 'launch_view_file_ext',
                                   'value_default' : ', '.join(nm.settings().LAUNCH_VIEW_EXT),
                                   'tooltip'  : '<p>Files that are displayed next to Launch '
                                      'files in the <span style="font-weight:600;">'
                                      'launch files</span> view</p>'
                                   },),
                'Store window layout:' : ({
                                   'value' : nm.settings().store_geometry,
                                   'settings' : nm.settings(),
                                   'attrname' : 'store_geometry',
                                   'value_default' : nm.settings().STORE_GEOMETRY,
                                   'tooltip'  : ''
                                   },),
                'Autoupdate:' : ({
                                   'value' : nm.settings().autoupdate,
                                   'settings' : nm.settings(),
                                   'attrname' : 'autoupdate',
                                   'value_default' : nm.settings().AUTOUPDATE,
                                   'tooltip'  : '<p>By default node manager updates the current '
                                      'state on changes. You can deactivate this behavior to '
                                      'reduce the network load. If autoupdate is deactivated '
                                      'you must refresh the state manually.</p>'
                                   },)
                }
    self.settings_model.init_settings(settings)
#    self.settingsTreeView.setSortingEnabled(True)
    self.settingsTreeView.sortByColumn(0, QtCore.Qt.AscendingOrder)
    self.settingsTreeView.expandAll()


class ItemDelegate(QtGui.QStyledItemDelegate):
  '''
  This ItemDelegate provides editors for different setting types in settings view.
  '''

  settings_path_changed_signal = QtCore.Signal()

  reload_settings = False

  def createEditor(self, parent, option, index):
    '''
    Creates a editor in the TreeView depending on type of the settings data.
    '''
    item = self._itemFromIndex(index)
    if item.edit_type() == SettingsValueItem.EDIT_TYPE_AUTODETECT:
      if isinstance(item.value(), bool):
        box = QtGui.QCheckBox(parent)
        box.setFocusPolicy(QtCore.Qt.StrongFocus)
        box.setAutoFillBackground(True)
        box.stateChanged.connect(self.edit_finished)
        return box
      elif isinstance(item.value(), int):
        box = QtGui.QSpinBox(parent)
        box.setValue(item.value())
        if not item.value_min() is None:
          box.setMinimum(item.value_min())
        if not item.value_max() is None:
          box.setMaximum(item.value_max())
        return box
    elif item.edit_type() == SettingsValueItem.EDIT_TYPE_FOLDER:
      editor = PathEditor(item.value(), parent)
      editor.editing_finished_signal.connect(self.edit_finished)
      return editor
    return QtGui.QStyledItemDelegate.createEditor(self, parent, option, index)

#  def setEditorData(self, editor, index):
#    print "setEditorData"
#    QtGui.QStyledItemDelegate.setEditorData(self, editor, index)

#  def updateEditorGeometry(self, editor, option, index):
#    print "updateEditorGeometry", option.rect.width()
#    editor.setMaximumSize(option.rect.width(), option.rect.height())
#    QtGui.QStyledItemDelegate.updateEditorGeometry(self, editor, option, index)

  def setModelData(self, editor, model, index):
    if isinstance(editor, PathEditor):
      cfg_path = nm.settings().cfg_path
      model.setData(index, editor.path)
      self.reload_settings = (cfg_path != nm.settings().cfg_path)
    else:
      QtGui.QStyledItemDelegate.setModelData(self, editor, model, index)

  def sizeHint(self, option, index):
    '''
    Determines and returns the size of the text after the format.
    @see: U{http://www.pyside.org/docs/pyside/PySide/QtGui/QAbstractItemDelegate.html#PySide.QtGui.QAbstractItemDelegate}
    '''
    options = QtGui.QStyleOptionViewItemV4(option)
    self.initStyleOption(options,index)
    return QtCore.QSize(options.rect.width(), 25)

  def edit_finished(self, arg=None):
    editor = self.sender()
    # The commitData signal must be emitted when we've finished editing
    # and need to write our changed back to the model.
    self.commitData.emit(editor)
    self.closeEditor.emit(editor, QtGui.QAbstractItemDelegate.NoHint)
    if self.reload_settings:
      self.reload_settings = False
      self.settings_path_changed_signal.emit()

  def _itemFromIndex(self, index):
    if isinstance(index.model(), QtGui.QSortFilterProxyModel):
      sindex = index.model().mapToSource(index)
      return index.model().sourceModel().itemFromIndex(sindex)
    else:
      return index.model().itemFromIndex(index)


class PathEditor(QtGui.QWidget):
  '''
  This is a path editor used as ItemDeligate in settings view. This editor
  provides an additional button for directory selection dialog.
  '''

  editing_finished_signal = QtCore.Signal()

  def __init__(self, path, parent=None):
    QtGui.QWidget.__init__(self, parent)
    self.path = path
    self._layout = QtGui.QHBoxLayout(self)
    self._layout.setContentsMargins(0, 0, 0, 0)
    self._layout.setSpacing(0)
    self._button = QtGui.QPushButton('...')
    self._button.setMaximumSize(QtCore.QSize(24, 20))
    self._button.clicked.connect(self._on_path_select_clicked)
    self._layout.addWidget(self._button)
    self._lineedit = QtGui.QLineEdit(path)
    self._lineedit.returnPressed.connect(self._on_editing_finished)
    self._layout.addWidget(self._lineedit)
    self.setLayout(self._layout)
    self.setFocusProxy(self._button)
    self.setAutoFillBackground(True)

  def _on_path_select_clicked(self):
    # Workaround for QtGui.QFileDialog.getExistingDirectory because it do not
    # select the configuration folder in the dialog
    self.dialog = QtGui.QFileDialog(self, caption='Select a new settings folder')
    self.dialog.setOption(QtGui.QFileDialog.HideNameFilterDetails, True)
    self.dialog.setFileMode(QtGui.QFileDialog.Directory)
    self.dialog.setDirectory(self.path)
    if self.dialog.exec_():
      fileNames = self.dialog.selectedFiles()
      path = fileNames[0]
      if os.path.isfile(path):
        path = os.path.basename(path)
      self._lineedit.setText(path)
      self.path = dir
      self.editing_finished_signal.emit()

  def _on_editing_finished(self):
    if self._lineedit.text():
      self.path = self._lineedit.text()
      self.editing_finished_signal.emit()
