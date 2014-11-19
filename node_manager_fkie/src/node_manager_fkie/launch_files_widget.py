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
#from datetime import datetime

from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding import loadUi

import rospy

from .common import package_name#, masteruri_from_ros
from .detailed_msg_box import WarningMessageBox
from .launch_list_model import LaunchListModel, LaunchItem
from .progress_queue import ProgressQueue#, ProgressThread
from .parameter_dialog import ParameterDialog

class LaunchFilesWidget(QtGui.QDockWidget):
  '''
  Launch file browser.
  '''

  load_signal = QtCore.Signal(str)
  ''' load the launch file '''
  load_as_default_signal = QtCore.Signal(str, str)
  ''' load the launch file as default (path, host) '''
  edit_signal = QtCore.Signal(list)
  ''' list of paths to open in an editor '''
  transfer_signal = QtCore.Signal(list)
  ''' list of paths selected for transfer '''

  def __init__(self, parent=None):
    '''
    Creates the window, connects the signals and init the class.
    '''
    QtGui.QDockWidget.__init__(self, parent)
    # initialize parameter
    self.__current_path = os.path.expanduser('~')
    # load the UI file
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'LaunchFilesDockWidget.ui')
    loadUi(ui_file, self)
    # initialize the view for the launch files
    self.launchlist_model = LaunchListModel()
    self.launchlist_proxyModel = QtGui.QSortFilterProxyModel(self)
    self.launchlist_proxyModel.setSourceModel(self.launchlist_model)
    self.xmlFileView.setModel(self.launchlist_proxyModel)
    self.xmlFileView.setAlternatingRowColors(True)
    self.xmlFileView.activated.connect(self.on_launch_selection_activated)
    self.xmlFileView.setDragDropMode(QtGui.QAbstractItemView.DragOnly)
    self.xmlFileView.setDragEnabled(True)
    sm = self.xmlFileView.selectionModel()
    sm.selectionChanged.connect(self.on_xmlFileView_selection_changed)
#    self.searchPackageLine.setVisible(False)
    self.searchPackageLine.textChanged.connect(self.set_package_filter)
    self.searchPackageLine.focusInEvent = self._searchline_focusInEvent
    # connect to the button signals
    self.refreshXmlButton.clicked.connect(self.on_refresh_xml_clicked)
    self.editXmlButton.clicked.connect(self.on_edit_xml_clicked)
    self.newXmlButton.clicked.connect(self.on_new_xml_clicked)
    self.openXmlButton.clicked.connect(self.on_open_xml_clicked)
    self.transferButton.clicked.connect(self.on_transfer_file_clicked)
    self.loadXmlButton.clicked.connect(self.on_load_xml_clicked)
    self.loadXmlAsDefaultButton.clicked.connect(self.on_load_as_default)
    # creates a default config menu
    start_menu = QtGui.QMenu(self)
    self.loadDeafaultAtHostAct = QtGui.QAction("&Load default config on host", self, statusTip="Loads the default config at given host", triggered=self.on_load_as_default_at_host)
    start_menu.addAction(self.loadDeafaultAtHostAct)
    self.loadXmlAsDefaultButton.setMenu(start_menu)
    #initialize the progress queue
    self.progress_queue = ProgressQueue(self.progressFrame_cfg, self.progressBar_cfg, self.progressCancelButton_cfg)

  def stop(self):
    '''
    Cancel the executing queued actions. This method must be
    called at the exit!
    '''
    self.progress_queue.stop()

  def on_launch_selection_activated(self, activated):
    '''
    Tries to load the launch file, if one was activated.
    '''
    selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
    for item in selected:
      try:
        lfile = self.launchlist_model.expandItem(item.name, item.path, item.id)
        self.searchPackageLine.setText('')
        if not lfile is None:
          if item.isLaunchFile():
            self.launchlist_model.add2LoadHistory(item.path)
            key_mod = QtGui.QApplication.keyboardModifiers()
            if (key_mod & QtCore.Qt.ShiftModifier):
              self.load_as_default_signal.emit(item.path, None)
            else:
              self.load_signal.emit(item.path)
          elif item.isConfigFile():
            self.edit_signal.emit([lfile])
      except Exception as e:
        rospy.logwarn("Error while load launch file %s: %s"%(item, e))
        WarningMessageBox(QtGui.QMessageBox.Warning, "Load error", 
                          'Error while load launch file:\n%s'%item.name,
                          "%s"%e).exec_()
#        self.launchlist_model.reloadCurrentPath()

  def on_xmlFileView_selection_changed(self, selected, deselected):
    '''
    On selection of a launch file, the buttons are enabled otherwise disabled.
    '''
    selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
    for item in selected:
      islaunch = item.isLaunchFile()
      isconfig = item.isConfigFile()
      self.editXmlButton.setEnabled(islaunch or isconfig)
      self.loadXmlButton.setEnabled(islaunch)
      self.transferButton.setEnabled(islaunch or isconfig)
      self.loadXmlAsDefaultButton.setEnabled(islaunch)

  def set_package_filter(self, text):
    self.launchlist_proxyModel.setFilterRegExp(QtCore.QRegExp(text,
                                                              QtCore.Qt.CaseInsensitive,
                                                              QtCore.QRegExp.Wildcard))

  def on_refresh_xml_clicked(self):
    '''
    Reload the current path.
    '''
    self.launchlist_model.reloadCurrentPath()
    self.launchlist_model.reloadPackages()
    self.editXmlButton.setEnabled(False)
    self.loadXmlButton.setEnabled(False)
    self.transferButton.setEnabled(False)
    self.loadXmlAsDefaultButton.setEnabled(False)

  def on_edit_xml_clicked(self):
    '''
    Opens an XML editor to edit the launch file. 
    '''
    selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
    for item in selected:
      path = self.launchlist_model.expandItem(item.name, item.path, item.id)
      if not path is None:
        self.edit_signal.emit([path])

  def on_new_xml_clicked(self):
    '''
    Creates a new launch file.
    '''
    # get new file from open dialog, use last path if one exists
    open_path = self.__current_path
    if not self.launchlist_model.currentPath is None:
      open_path = self.launchlist_model.currentPath
    (fileName, _) = QtGui.QFileDialog.getSaveFileName(self,
                                                 "New launch file",
                                                 open_path,
                                                 "Config files (*.launch *.yaml);;All files (*)")
    if fileName:
      try:
        (pkg, _) = package_name(os.path.dirname(fileName))#_:=pkg_path
        if pkg is None:
          WarningMessageBox(QtGui.QMessageBox.Warning, "New File Error", 
                         'The new file is not in a ROS package').exec_()
          return
        with open(fileName, 'w+') as f:
          f.write("<launch>\n"
                  "  <arg name=\"robot_ns\" default=\"my_robot\"/>\n"
                  "  <group ns=\"$(arg robot_ns)\">\n"
                  "    <param name=\"tf_prefix\" value=\"$(arg robot_ns)\"/>\n"
                  "\n"
                  "    <node pkg=\"my_pkg\" type=\"my_node\" name=\"my_name\" >\n"
                  "      <param name=\"capability_group\" value=\"MY_GROUP\"/>\n"
                  "    </node>\n"
                  "  </group>\n"
                  "</launch>\n"
                  )
        self.__current_path = os.path.dirname(fileName)
        self.launchlist_model.setPath(self.__current_path)
        self.edit_signal.emit([fileName])
      except EnvironmentError as e:
        WarningMessageBox(QtGui.QMessageBox.Warning, "New File Error", 
                         'Error while create a new file',
                          '%s'%e).exec_()

  def on_open_xml_clicked(self):
    (fileName, _) = QtGui.QFileDialog.getOpenFileName(self,
                                                 "Load launch file", 
                                                 self.__current_path, 
                                                 "Config files (*.launch);;All files (*)")
    if fileName:
      self.__current_path = os.path.dirname(fileName)
      self.launchlist_model.add2LoadHistory(fileName)
      self.load_signal.emit(fileName)

  def on_transfer_file_clicked(self):
    '''
    Emit the signal to copy the selected file to a remote host.
    '''
    selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
    paths = list()
    for item in selected:
      path = self.launchlist_model.expandItem(item.name, item.path, item.id)
      if not path is None:
        paths.append(path)
    if paths:
      self.transfer_signal.emit(paths)

  def on_load_xml_clicked(self):
    '''
    Tries to load the selected launch file. The button is only enabled and this
    method is called, if the button was enabled by on_launch_selection_clicked()
    '''
    selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
    for item in selected:
      path = self.launchlist_model.expandItem(item.name, item.path, item.id)
      if not path is None:
        self.launchlist_model.add2LoadHistory(path)
        self.load_signal.emit(path)

  def on_load_as_default_at_host(self):
    '''
    Tries to load the selected launch file as default configuration. The button 
    is only enabled and this method is called, if the button was enabled by 
    on_launch_selection_clicked()
    '''
    selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
    for item in selected:
      path = self.launchlist_model.expandItem(item.name, item.path, item.id)
      if not path is None:
        params = {'Host' : ('string', 'localhost') }
        dia = ParameterDialog(params)
        dia.setFilterVisible(False)
        dia.setWindowTitle('Start node on...')
        dia.resize(350,120)
        dia.setFocusField('Host')
        if dia.exec_():
          try:
            params = dia.getKeywords()
            host = params['Host']
            rospy.loginfo("LOAD the launch file on host %s as default: %s"%(host, path))
            self.launchlist_model.add2LoadHistory(path)
            self.load_as_default_signal.emit(path, host)
          except Exception, e:
            WarningMessageBox(QtGui.QMessageBox.Warning, "Load default config error", 
                             'Error while parse parameter',
                              '%s'%e).exec_()


  def on_load_as_default(self):
    '''
    Tries to load the selected launch file as default configuration. The button 
    is only enabled and this method is called, if the button was enabled by 
    on_launch_selection_clicked()
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if (key_mod & QtCore.Qt.ShiftModifier):
      self.loadXmlAsDefaultButton.showMenu()
    else:
      selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
      for item in selected:
        path = self.launchlist_model.expandItem(item.name, item.path, item.id)
        if not path is None:
          rospy.loginfo("LOAD the launch file as default: %s", path)
          self.launchlist_model.add2LoadHistory(path)
          self.load_as_default_signal.emit(path, None)

  def _launchItemsFromIndexes(self, indexes, recursive=True):
    result = []
    for index in indexes:
      if index.column() == 0:
        model_index = self.launchlist_proxyModel.mapToSource(index)
        item = self.launchlist_model.itemFromIndex(model_index)
        if not item is None and isinstance(item, LaunchItem):
          result.append(item)
    return result

  def keyReleaseEvent(self, event):
    '''
    Defines some of shortcuts for navigation/management in launch
    list view or topics view.
    '''
    key_mod = QtGui.QApplication.keyboardModifiers()
    if not self.xmlFileView.state() == QtGui.QAbstractItemView.EditingState:
      # remove history file from list by pressing DEL
      if event == QtGui.QKeySequence.Delete:
        selected = self._launchItemsFromIndexes(self.xmlFileView.selectionModel().selectedIndexes(), False)
        for item in selected:
          if item.path in self.launchlist_model.load_history:
            self.launchlist_model.removeFromLoadHistory(item.path)
            self.launchlist_model.reloadCurrentPath()
      elif not key_mod and event.key() == QtCore.Qt.Key_F4 and self.editXmlButton.isEnabled():
        # open selected launch file in xml editor by F4
        self.on_edit_xml_clicked()
      elif event == QtGui.QKeySequence.Find:
        # set focus to filter box for packages
        self.searchPackageLine.setFocus(QtCore.Qt.ActiveWindowFocusReason)
      elif event == QtGui.QKeySequence.Paste:
        # paste files from clipboard
        self.launchlist_model.paste_from_clipboard()
      elif event == QtGui.QKeySequence.Copy:
        # copy the selected items as file paths into clipboard
        selected = self.xmlFileView.selectionModel().selectedIndexes()
        indexes = []
        for s in selected:
          indexes.append(self.launchlist_proxyModel.mapToSource(s))
        self.launchlist_model.copy_to_clipboard(indexes)
    if self.searchPackageLine.hasFocus() and event.key() == QtCore.Qt.Key_Escape:
      # cancel package filtering on pressing ESC 
      self.launchlist_model.show_packages(False)
      self.searchPackageLine.setText('')
      self.xmlFileView.setFocus(QtCore.Qt.ActiveWindowFocusReason)
    QtGui.QDockWidget.keyReleaseEvent(self, event)

  def _searchline_focusInEvent(self, event):
    self.launchlist_model.show_packages(True)
    QtGui.QLineEdit.focusInEvent(self.searchPackageLine, event)