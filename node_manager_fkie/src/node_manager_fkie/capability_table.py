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

import rospy
import node_manager_fkie as nm
from common import resolve_paths


################################################################################
##############                  CapabilityHeader                  ##############
################################################################################

class CapabilityHeader(QtGui.QHeaderView):
  '''
  This class is used for visualization of robots or capabilities in header of 
  the capability table. It is also used to manage the displayed robots or
  capabilities. Furthermore L{QtGui.QHeaderView.paintSection()} method is
  overridden to paint the images in background of the cell.
  '''

  description_requested_signal = QtCore.Signal(str, str)
  '''the signal is emitted by click on a header to show a description.'''

  def __init__(self, orientation, parent=None):
    QtGui.QHeaderView.__init__(self, orientation, parent)
    self._data = []
    ''' @ivar a list with dictionaries C{dict('cfgs' : [], 'name': str, 'displayed_name' : str, 'type' : str, 'description' : str, 'images' : [QtGui.QPixmap])}'''
    if orientation == QtCore.Qt.Horizontal:
      self.setDefaultAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom)
    elif orientation == QtCore.Qt.Vertical:
      self.setDefaultAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
    self.controlWidget = []

  def index(self, name):
    '''
    Returns the index of the object stored with given name
    @param name: the name of the item
    @type name: C{str}
    @return: the index or -1 if the item was not found
    @rtype: C{int}
    '''
    for index, entry in enumerate(self._data):
      if entry['name'] == name:
        return index
    return -1

  def paintSection(self, painter, rect, logicalIndex):
    '''
    The method paint the robot or capability images in the backgroud of the cell.
    @see: L{QtGui.QHeaderView.paintSection()} 
    '''
    painter.save()
    QtGui.QHeaderView.paintSection(self, painter, rect, logicalIndex)
    painter.restore()

    if logicalIndex in range(len(self._data)) and self._data[logicalIndex]['images']:
      if len(self._data[logicalIndex]['images']) == 1:
        pix = self._data[logicalIndex]['images'][0]
        pix = pix.scaled(rect.width(), rect.height()-20, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.style().drawItemPixmap(painter, rect, 5, pix)
      elif len(self._data[logicalIndex]['images']) > 1:
        new_rect = QtCore.QRect(rect.left(), rect.top(), rect.width(), (rect.height()-20) / 2.)
        pix = self._data[logicalIndex]['images'][0]
        pix = pix.scaled(new_rect.width(), new_rect.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.style().drawItemPixmap(painter, new_rect, 5, pix)
        new_rect = QtCore.QRect(rect.left(), rect.top()+new_rect.height(), rect.width(), new_rect.height())
        pix = self._data[logicalIndex]['images'][1]
        pix = pix.scaled(new_rect.width(), new_rect.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.style().drawItemPixmap(painter, new_rect, 5, pix)

  def mousePressEvent(self, event):
    '''
    Interpret the mouse events to send the description of a robot or capability 
    if the user click on the header.
    '''
    QtGui.QHeaderView.mousePressEvent(self, event)
    index = self.logicalIndexAt(event.pos())
    if index in range(len(self._data)):
      suffix = 'Capability'
      if self.orientation() == QtCore.Qt.Horizontal:
        suffix = 'Robot'
      title = ' - '.join([self._data[index]['name'], suffix])
      text = self._data[index]['description']
      try:
        from docutils import examples
        text = examples.html_body(text)
      except:
        import traceback
        rospy.logwarn("Error while generate description for %s: %s", self._data[index]['name'], traceback.format_exc(1))
      self.description_requested_signal.emit(title, text)

  def setDescription(self, index, cfg, name, displayed_name, robot_type, description, images):
    '''
    Sets the values of an existing item to the given items.
    '''
    if index < len(self._data):
      obj = self._data[index]
      if not cfg in obj['cfgs']:
        obj['cfgs'].append(cfg)
      obj['name'] = name
      obj['displayed_name'] = displayed_name
      obj['type'] = robot_type
      obj['description'] = resolve_paths(description)
      del obj['images'][:]
      for image_path in images:
        img = resolve_paths(image_path)
        if img and img[0] != os.path.sep:
          img = os.path.join(nm.settings().PACKAGE_DIR, image_path)
        if os.path.isfile(img):
          obj['images'].append(QtGui.QPixmap(img))

  def updateDescription(self, index, cfg, name, displayed_name, robot_type, description, images):
    '''
    Sets the values of an existing item to the given items only if the current 
    value is empty.
    '''
    if index < len(self._data):
      obj = self._data[index]
      if not cfg in obj['cfgs']:
        obj['cfgs'].append(cfg)
      if not obj['name']:
        obj['name'] = name
      if not obj['displayed_name']:
        obj['displayed_name'] = displayed_name
      if not obj['type']:
        obj['type'] = robot_type
      if not obj['description']:
        obj['description'] = resolve_paths(description)
      if not obj['images']:
        for image_path in images:
          img = resolve_paths(image_path)
          if img and img[0] != os.path.sep:
            img = os.path.join(nm.settings().PACKAGE_DIR, image_path)
          if os.path.isfile(img):
            obj['images'].append(QtGui.QPixmap(img))

  def removeDescription(self, index):
    '''
    Removes an existing value from the header.
    @param index: the index of the item to remove.
    @type index: C{int}
    '''
    if index < len(self._data):
      self._data.pop(index)
    
  def insertItem(self, index):
    '''
    Inserts an item at the given index into the header.
    @param index: the index
    @type index: C{int}
    '''
    new_dict = {'cfgs' : [], 'name': '', 'displayed_name' : '', 'type' : '', 'description' : '', 'images' : []}
    if index < len(self._data):
      self._data.insert(index, new_dict)
    else:
      self._data.append(new_dict)

  def insertSortedItem(self, name, displayed_name):
    '''
    Insert the new item with given name at the sorted position and return the index of
    the item.
    @param name: the name of the new item
    @type name: C{str}
    @return: index of the inserted item
    @rtype: C{int}
    '''
    new_dict = {'cfgs' : [], 'name': name, 'displayed_name' : displayed_name, 'type' : '', 'description' : '', 'images' : []}
    for index, item in enumerate(self._data):
      if item['displayed_name'].lower() > displayed_name.lower():
        self._data.insert(index, new_dict)
        return index
    self._data.append(new_dict)
    return len(self._data)-1

  def removeCfg(self, cfg):
    '''
    Removes the configuration entries from objects and returns the list with 
    indexes, where the configuration was removed.
    @param cfg: configuration to remove
    @type cfg: C{str}
    @return: the list the indexes, where the configuration was removed
    @rtype: C{[int]}
    '''
    result = []
    for index, d in enumerate(self._data):
      if cfg in d['cfgs']:
        d['cfgs'].remove(cfg)
        result.append(index)
    return result
  
  def count(self):
    '''
    @return: The count of items in the header.
    @rtype: C{int}
    '''
    return len(self._data)

  def getConfigs(self, index):
    '''
    @return: The configurations assigned to the item at the given index
    @rtype: C{str}
    '''
    result = []
    if index < len(self._data):
      result = list(self._data[index]['cfgs'])
    return result


################################################################################
##############              CapabilityControlWidget               ##############
################################################################################

class CapabilityControlWidget(QtGui.QFrame):
  '''
  The control widget contains buttons for control a capability. Currently this 
  are C{On} and C{Off} buttons. Additionally, the state of the capability is 
  color coded.
  '''

  start_nodes_signal = QtCore.Signal(str, str, list)
  '''@ivar: the signal is emitted to start on host(described by masteruri) the nodes described in the list, Parameter(masteruri, config, nodes).'''

  stop_nodes_signal = QtCore.Signal(str, list)
  '''@ivar: the signal is emitted to stop on masteruri the nodes described in the list.'''

  def __init__(self, masteruri, cfg, ns, nodes, parent=None):
    QtGui.QFrame.__init__(self, parent)
    self._masteruri = masteruri
    self._nodes = {cfg : {ns : nodes} }
    frame_layout = QtGui.QVBoxLayout(self)
    frame_layout.setContentsMargins(0, 0, 0, 0)
    # create frame for warning label
    self.warning_frame = warning_frame = QtGui.QFrame()
    warning_layout = QtGui.QHBoxLayout(warning_frame)
    warning_layout.setContentsMargins(0, 0, 0, 0)
    warning_layout.addItem(QtGui.QSpacerItem(0, 0, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding))
    self.warning_label = QtGui.QLabel()
    icon = QtGui.QIcon(':/icons/crystal_clear_warning.png')
    self.warning_label.setPixmap(icon.pixmap(QtCore.QSize(40, 40)))
    self.warning_label.setToolTip('Multiple configuration for same node found!\nA first one will be selected for the start a node!')
    warning_layout.addWidget(self.warning_label)
    warning_layout.addItem(QtGui.QSpacerItem(0, 0, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding))
    frame_layout.addItem(QtGui.QSpacerItem(0, 0, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding))
    frame_layout.addWidget(warning_frame)
    # create frame for start/stop buttons
    buttons_frame = QtGui.QFrame()
    buttons_layout = QtGui.QHBoxLayout(buttons_frame)
    buttons_layout.setContentsMargins(0, 0, 0, 0)
    buttons_layout.addItem(QtGui.QSpacerItem(20, 20))
    self.on_button = QtGui.QPushButton()
    self.on_button.setFlat(False)
    self.on_button.setText("On")
    self.on_button.clicked.connect(self.on_on_clicked)
    buttons_layout.addWidget(self.on_button)

    self.off_button = QtGui.QPushButton()
    self.off_button.setFlat(True)
    self.off_button.setText("Off")
    self.off_button.clicked.connect(self.on_off_clicked)
    buttons_layout.addWidget(self.off_button)
    buttons_layout.addItem(QtGui.QSpacerItem(20, 20))
    frame_layout.addWidget(buttons_frame)
    frame_layout.addItem(QtGui.QSpacerItem(0, 0, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding))
    self.warning_frame.setVisible(False)

  def hasConfigs(self):
    '''
    @return: True, if a configurations for this widget are available.
    @rtype: bool
    '''
    return len(self._nodes) > 0

  def nodes(self, cfg=''):
    '''
    @return: the list with nodes required by this capability. The nodes are 
    defined by ROS full name.
    @rtype: C{[str]}
    '''
    try:
      if cfg:
        return [n for l in self._nodes[cfg].itervalues() for n in l]
      else:
        return [n for c in self._nodes.itervalues() for l in c.itervalues() for n in l]
    except:
      return []

  def setNodeState(self, running_nodes, stopped_nodes, error_nodes):
    '''
    Sets the state of this capability.
    @param running_nodes: a list with running nodes.
    @type running_nodes: C{[str]}
    @param stopped_nodes: a list with not running nodes.
    @type stopped_nodes: C{[str]}
    @param error_nodes: a list with nodes having a problem.
    @type error_nodes: C{[str]}
    '''
    self.setAutoFillBackground(True)
    self.setBackgroundRole(QtGui.QPalette.Base)
    palette = QtGui.QPalette()
    if error_nodes:
      brush = QtGui.QBrush(QtGui.QColor(255, 100, 0))
    elif running_nodes and stopped_nodes:
      brush = QtGui.QBrush(QtGui.QColor(140, 185, 255)) #30, 50, 255
    elif running_nodes:
      self.on_button.setFlat(True)
      self.off_button.setFlat(False)
      brush = QtGui.QBrush(QtGui.QColor(59, 223, 18)) # 59, 223, 18
    else:
      brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
      self.on_button.setFlat(False)
      self.off_button.setFlat(True)
    palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
    brush.setStyle(QtCore.Qt.SolidPattern)
    palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
    self.setPalette(palette)

  def removeCfg(self, cfg):
    try:
      del self._nodes[cfg]
    except:
      pass

  def updateNodes(self, cfg, ns, nodes):
    self._nodes[cfg] = {ns : nodes}
    test_nodes = self.nodes()
    self.warning_frame.setVisible(len(test_nodes) != len(set(test_nodes)))

  def on_on_clicked(self):
    started = set() # do not start nodes multiple times
    for c in self._nodes.iterkeys():
      node2start = set(self.nodes(c)) - started
      self.start_nodes_signal.emit(self._masteruri, c, list(node2start))
      started.update(node2start)
    self.on_button.setFlat(True)
    self.off_button.setFlat(False)

  def on_off_clicked(self):
    self.stop_nodes_signal.emit(self._masteruri, self.nodes())
    self.on_button.setFlat(False)
    self.off_button.setFlat(True)



################################################################################
##############                  CapabilityTable                   ##############
################################################################################

class CapabilityTable(QtGui.QTableWidget):
  '''
  The table shows all detected capabilities of robots in tabular view. The 
  columns represents the robot and rows the capabilities. The cell of available 
  capability contains a L{CapabilityControlWidget} to show the state and manage 
  the capability.
  '''
  
  start_nodes_signal = QtCore.Signal(str, str, list)
  '''@ivar: the signal is emitted to start on host(described by masteruri) the nodes described in the list, Parameter(masteruri, config, nodes).'''

  stop_nodes_signal = QtCore.Signal(str, list)
  '''@ivar: the signal is emitted to stop on masteruri the nodes described in the list.'''

  description_requested_signal = QtCore.Signal(str, str)
  '''@ivar: the signal is emitted by click on a header to show a description.'''


  def __init__(self, parent=None):
    QtGui.QTableWidget.__init__(self, parent)
    self._robotHeader = CapabilityHeader(QtCore.Qt.Horizontal, self)
    self._robotHeader.description_requested_signal.connect(self._show_description)
    self.setHorizontalHeader(self._robotHeader)
    self._capabilityHeader = CapabilityHeader(QtCore.Qt.Vertical, self)
    self._capabilityHeader.description_requested_signal.connect(self._show_description)
    self.setVerticalHeader(self._capabilityHeader)

  def updateCapabilities(self, masteruri, cfg_name, description):
    '''
    Updates the capabilities view.
    @param masteruri: the ROS master URI of updated ROS master.
    @type masteruri: C{str}
    @param cfg_name: The name of the node provided the capabilities description.
    @type cfg_name: C{str}
    @param description: The capabilities description object.
    @type description: L{default_cfg_fkie.Description}
    '''
    # if it is a new masteruri add a new column
    robot_index = self._robotHeader.index(masteruri)
    robot_name = description.robot_name if description.robot_name else nm.nameres().mastername(masteruri)
    if robot_index == -1:
      # append a new robot
      robot_index = self._robotHeader.insertSortedItem(masteruri, robot_name)
      self.insertColumn(robot_index)
#      robot_index = self.columnCount()-1
#      self._robotHeader.insertItem(robot_index)
      self._robotHeader.setDescription(robot_index, cfg_name, masteruri, robot_name, description.robot_type, description.robot_descr.replace("\\n ", "\n").decode(sys.getfilesystemencoding()), description.robot_images)
      item = QtGui.QTableWidgetItem()
      item.setSizeHint(QtCore.QSize(96,96))
      self.setHorizontalHeaderItem(robot_index, item)
      self.horizontalHeaderItem(robot_index).setText(robot_name)
    else:
      #update
      self._robotHeader.setDescription(robot_index, cfg_name, masteruri, robot_name, description.robot_type, description.robot_descr.replace("\\n ", "\n").decode(sys.getfilesystemencoding()), description.robot_images)
    
    #set the capabilities
    for c in description.capabilities:
      cap_index = self._capabilityHeader.index(c.name.decode(sys.getfilesystemencoding()))
      if cap_index == -1:
        # append a new capability
        cap_index = self._capabilityHeader.insertSortedItem(c.name.decode(sys.getfilesystemencoding()), c.name.decode(sys.getfilesystemencoding()))
        self.insertRow(cap_index)
        self.setRowHeight(cap_index, 96)
        self._capabilityHeader.setDescription(cap_index, cfg_name, c.name.decode(sys.getfilesystemencoding()), c.name.decode(sys.getfilesystemencoding()), c.type, c.description.replace("\\n ", "\n").decode(sys.getfilesystemencoding()), c.images)
        item = QtGui.QTableWidgetItem()
        item.setSizeHint(QtCore.QSize(96,96))
        self.setVerticalHeaderItem(cap_index, item)
        self.verticalHeaderItem(cap_index).setText(c.name.decode(sys.getfilesystemencoding()))
        # add the capability control widget
        controlWidget = CapabilityControlWidget(masteruri, cfg_name, c.namespace, c.nodes)
        controlWidget.start_nodes_signal.connect(self._start_nodes)
        controlWidget.stop_nodes_signal.connect(self._stop_nodes)
        self.setCellWidget(cap_index, robot_index, controlWidget)
        self._capabilityHeader.controlWidget.insert(cap_index, controlWidget)
      else:
        self._capabilityHeader.updateDescription(cap_index, cfg_name, c.name.decode(sys.getfilesystemencoding()), c.name.decode(sys.getfilesystemencoding()), c.type, c.description.replace("\\n ", "\n").decode(sys.getfilesystemencoding()), c.images)
        self._capabilityHeader.controlWidget[cap_index].updateNodes(cfg_name, c.namespace, c.nodes)

  def removeConfig(self, cfg):
    '''
    @param cfg: The name of the node provided the capabilities description.
    @type cfg: C{str}
    '''
    removed_from_robots = self._robotHeader.removeCfg(cfg)
#    for r in removed_from_robots:
#      if not self._robotHeader.getConfigs(r):
#        #remove the column with robot
#        pass
    removed_from_caps = self._capabilityHeader.removeCfg(cfg)
    # remove control widget with given configuration
    for r in reversed(removed_from_robots):
      for c in removed_from_caps:
        controlWidget = self.cellWidget(c, r)
        if isinstance(controlWidget, CapabilityControlWidget):
          controlWidget.removeCfg(cfg)
          if not controlWidget.hasConfigs():
            self.removeCellWidget(c, r)
    # remove empty columns
    for r in removed_from_robots:
      is_empty = True
      for c in reversed(range(self.rowCount())):
        controlWidget = self.cellWidget(c, r)
        if isinstance(controlWidget, CapabilityControlWidget):
          is_empty = False
          break
      if is_empty:
        self.removeColumn(r)
        self._robotHeader.removeDescription(r)
    # remove empty rows
    for c in reversed(removed_from_caps):
      is_empty = True
      for r in reversed(range(self.columnCount())):
        controlWidget = self.cellWidget(c, r)
        if isinstance(controlWidget, CapabilityControlWidget):
          is_empty = False
          break
      if is_empty:
        self.removeRow(c)
        self._capabilityHeader.removeDescription(c)

  def updateState(self, masteruri, master_info):
    '''
    Updates the run state of the capability.
    @param masteruri: The ROS master, which sends the master_info
    @type masteruri: C{str}
    @param master_info: The state of the ROS master
    @type master_info: L{master_discovery_fkie.MasterInfo}
    '''
    if master_info is None or masteruri is None:
      return
    robot_index = self._robotHeader.index(masteruri)
    if robot_index != -1:
      for c in range(self.rowCount()):
        controlWidget = self.cellWidget(c, robot_index)
        if not controlWidget is None:
          running_nodes = []
          stopped_nodes = []
          error_nodes = []
          for n in controlWidget.nodes():
            node = master_info.getNode(n)
            if not node is None:
              # while a synchronization there are node from other hosts in the master_info -> filter these nodes
              if not node.uri is None and masteruri == node.masteruri:
                if not node.pid is None:
                  running_nodes.append(n)
                else:
                  error_nodes.append(n)
            else:
              stopped_nodes.append(n)
          controlWidget.setNodeState(running_nodes, stopped_nodes, error_nodes)

  def _start_nodes(self, masteruri, cfg, nodes):
    self.start_nodes_signal.emit(masteruri, cfg, nodes)

  def _stop_nodes(self, masteruri, nodes):
    self.stop_nodes_signal.emit(masteruri, nodes)

  def _show_description(self, name, description):
    self.description_requested_signal.emit(name, description)
      
