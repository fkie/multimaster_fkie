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

from .settings_model import SettingsModel


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
#    self.item_delegate = ItemDelegate()
#    self.settingsTreeView.setItemDelegateForColumn(1, self.item_delegate)
    settings = {#'General' : {
                   'default_user' : (nm.settings().default_user, 'Default user:', '', nm.settings()),
                   'launch_history_length' : (nm.settings().launch_history_length, 
                                              'Launch history length:',
                                    '<html><head/><body><p>The count of recent '
                                    'loaded launch files displayed in the root '
                                    'of the <span style=" font-weight:600;">launch '
                                    'files</span> view.</p></body></html>', nm.settings()),
                   'param_history_length' : (nm.settings().param_history_length, 
                                             'Param history length:',
                                    '<html><head/><body><p>The count of parameters stored which '
                                    'are entered in a parameter dialog (Launch file arguments, '
                                    'paramter server, publishing to a topic, service call)</p>'
                                    '</body></html>', nm.settings())
#                            }
                            ,
#                'Paths'   : {
                   'cfg_path' : (nm.settings().cfg_path, 'Settings path:', '', nm.settings()),
                   'robots_path' : (nm.settings().robots_path, 'Robot icon path:',
                                    '<html><head/><body><p>The path to the folder with '
                                    'robot images (<span style=" font-weight:600;">.png</span>).'
                                    'The images with robot name will be displayed in the info '
                                    'bar.</p></body></html>', nm.settings())
#                            }
                             ,
#                'File extensions' : {
                   'launch_view_file_ext' : (', '.join(nm.settings().launch_view_file_ext),
                                             'View file extentions:',
                            '<html><head/><body><p>Files displayed in <span style=" '
                            'font-weight:600;">launch files</span> view</p></body></html>', nm.settings()),
                   'follow_include_file_ext' : (', '.join(nm.settings().follow_include_file_ext),
                                              'Follow file extentions:',
                            '<html><head/><body><p>Files which are classified as '
                            '<span style=" font-weight:600;">included file</span> '
                            'in launch files.</p></body></html>', nm.settings())
#                            }
                            ,
#                'GUI'     : {
                   'store_geometry' : (nm.settings().store_geometry, 'Store window layout:', '', nm.settings())
#                            }
                }
    self.settings_model.init_settings(settings)
#    self.settingsTreeView.setSortingEnabled(True)
    self.settingsTreeView.sortByColumn(0, QtCore.Qt.AscendingOrder)
    self.settingsTreeView.expandAll()



# class ItemDelegate(QtGui.QStyledItemDelegate):
# 
#   ID_PATH = 1
# 
#   def createEditor(self, parent, option, index):
#     options = QtGui.QStyleOptionViewItemV4(option)
#     self.initStyleOption(options, index)
#     sindex = index.model().mapToSource(index)
#     item = index.model().sourceModel().itemFromIndex(sindex)
#     print "index type:", index.model().sourceModel().itemFromIndex(sindex).type()
#     #item.setText('')
#     box = QtGui.QLineEdit(parent)
# #    box.setChecked(item.value())
#     box.setMaximumSize(option.rect.width(), option.rect.height())
# #    return box
#     return QtGui.QStyledItemDelegate.createEditor(self, parent, option, index)
#   def setEditorData(self, editor, index):
#     pass
#     print "setEditorData"
#     QtGui.QStyledItemDelegate.setEditorData(self, editor, index)
# #  def updateEditorGeometry(self, editor, option, index):
# #    pass
# #    print "updateEditorGeometry", option.rect.width()
# ##    editor
# #    #editor.setMaximumSize(option.rect.height()-4, option.rect.height()-4)
# #    QtGui.QStyledItemDelegate.updateEditorGeometry(self, editor, option, index)
#   def setModelData(self, editor, model, index):
#     print "setModelData"
#     QtGui.QStyledItemDelegate.setModelData(self, editor, model, index)
#   
# #   def sizeHint(self, option, index):
# #     '''
# #     Determines and returns the size of the text after the format.
# #     @see: U{http://www.pyside.org/docs/pyside/PySide/QtGui/QAbstractItemDelegate.html#PySide.QtGui.QAbstractItemDelegate}
# #     '''
# #     options = QtGui.QStyleOptionViewItemV4(option)
# #     self.initStyleOption(options,index)
# # 
# #     return QtCore.QSize(options.rect.width(), options.rect.height())
  