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
from datetime import datetime

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
    for i, (name, width) in enumerate(SettingsModel.header):
      self.settingsTreeView.setColumnWidth(i, width)
    self.item_delegate = ItemDelegate()
    self.settingsTreeView.setItemDelegateForColumn(1, self.item_delegate)
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
                'View file extentions:' : ({
                                   'value' : ', '.join(nm.settings().launch_view_file_ext),
                                   'settings' : nm.settings(),
                                   'attrname' : 'launch_view_file_ext',
                                   'value_default' : ', '.join(nm.settings().LAUNCH_VIEW_EXT),
                                   'tooltip'  : '<p>Files displayed in <span style=" '
                                      'font-weight:600;">launch files</span> view</p>'
                                   },),
                'Follow file extentions:' : ({
                                   'value' : ', '.join(nm.settings().follow_include_file_ext),
                                   'settings' : nm.settings(),
                                   'attrname' : 'launch_view_file_ext',
                                   'value_default' : ', '.join(nm.settings().FOLLOW_INCLUDED_EXT),
                                   'tooltip'  : '<p>Files which are classified as '
                                      '<span style=" font-weight:600;">included file</span> '
                                      'in launch files.</p>'
                                   },),

                'Store window layout:' : ({
                                   'value' : nm.settings().store_geometry,
                                   'settings' : nm.settings(),
                                   'attrname' : 'store_geometry',
                                   'value_default' : nm.settings().STORE_GEOMETRY,
                                   'tooltip'  : ''
                                   },)
                }
    self.settings_model.init_settings(settings)
#    self.settingsTreeView.setSortingEnabled(True)
    self.settingsTreeView.sortByColumn(0, QtCore.Qt.AscendingOrder)
    self.settingsTreeView.expandAll()



class ItemDelegate(QtGui.QStyledItemDelegate):

  def createEditor(self, parent, option, index):
    item = self._itemFromIndex(index)
    if item.edit_type() == SettingsValueItem.EDIT_TYPE_AUTODETECT:
      if isinstance(item.value(), bool):
        item.setData('1' if item.value() else '0', QtCore.Qt.EditRole)
        box = QtGui.QCheckBox(parent)
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
      #TODO
#      editor = PathEditor(item.value())
#      return editor
      pass
    return QtGui.QStyledItemDelegate.createEditor(self, parent, option, index)

  def setEditorData(self, editor, index):
#    print "setEditorData"
    QtGui.QStyledItemDelegate.setEditorData(self, editor, index)
#     if index.column() == 3:
#       editor.starRating = StarRating(index.data())
#     else:
#       QStyledItemDelegate.setModelData(self, editor, model, index)

#  def updateEditorGeometry(self, editor, option, index):
#    pass
#    print "updateEditorGeometry", option.rect.width()
##    editor
#    editor.setMaximumSize(25, 25)  
#    QtGui.QStyledItemDelegate.updateEditorGeometry(self, editor, option, index)

  def setModelData(self, editor, model, index):
#    print "setModelData"
    QtGui.QStyledItemDelegate.setModelData(self, editor, model, index)
#     if index.column() == 3:
#       model.setData(index, editor.starRating.starCount)
#     else:
#       QStyledItemDelegate.setModelData(self, editor, model, index)
  
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
            
  def _itemFromIndex(self, index):
    if isinstance(index.model(), QtGui.QSortFilterProxyModel):
      sindex = index.model().mapToSource(index)
      return index.model().sourceModel().itemFromIndex(sindex)
    else:
      return index.model().itemFromIndex(index)

class PathEditor(QtGui.QFrame):

  def __init__(self, path, parent=None):
    QtGui.QWidget.__init__(self, parent)
    self.path = path
    self._layout = QtGui.QHBoxLayout(self)
    self._button = QtGui.QPushButton('...')
    self._button.clicked.connect(self._on_path_select_clicked)
    self._layout.addWidget(self._button)
    self.setLayout(self._layout)
   
  def _on_path_select_clicked(self):
    dialog = QFileDialog(self)
    dialog.setFileMode(QFileDialog.Directory)
    dialog.setDirectory(self._path)
    if dialog.exce_():
      fileNames = dialog.selectedFiles()
      print "files", fileNames
    
   
