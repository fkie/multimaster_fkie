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
#  * Neither the name of I Heart Engineering nor the names of its
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

from urlparse import urlparse

from PySide import QtCore
from PySide import QtGui

import node_manager_fkie as nm

class ExtendedNodeInfo(object):
  '''
  The ExtendedNodeInfo stores all information about a node. This information is 
  used to create the view items.
  '''
  
  def __init__(self, nodename, mastername, masteruri, uri=None, pid=None, published=[], subscribed=[], services=[], cfgs=[], default_cfgs=[]):
    '''
    Initialize the ExtendedNodeInfo object with given values.
    @param nodename: the name of the node
    @type nodename: C{str}
    @param mastername: the name of ROS master, where the node is registered
    @type mastername: C{str}
    @param masteruri: the URI of ROS master, where the node is registered
    @type masteruri: C{str}
    @param uri: the URI of of the node, if the node was started and registered 
    by ROS master.
    @type uri: C{str} (Default: C{None})
    @param pid: the process ID of of the node, if the node is running and is 
    located at the same host as the "main" ROS master.
    @type pid: C{int} (Default: C{None})
    @param published: the list with all topics published by this node.
    @type published: C{[str, ...]} (Default: C{[]})
    @param subscribed: the list with all topics subscribed by this node.
    @type subscribed: C{[str, ...]} (Default: C{[]})
    @param services: the list with all services provided by this node.
    @type services: C{[str, ...]} (Default: C{[]})
    @param cfgs: the list with all loaded configuration, which contains this node.
    @type cfgs: C{[str, ...]} (Default: C{[]})
    @param default_cfgs: the list with all default configurations, which contains this node. @see: X{http://ros.org/wiki/default_cfg_fkie|default_cfg_fkie}
    @type default_cfgs: C{[str, ...]} (Default: C{[]})
    '''
    self.name = nodename
    self.mastername = mastername
    self.masteruri = masteruri
    self.uri = uri
    self.pid = pid
    self.published = published
    self.subscribed = subscribed
    self.services = services
    self.cfgs = cfgs
    self.default_cfgs = default_cfgs
    self.descr_type = self.descr_name = self.descr = ''
  
  def addConfig(self, cfgs):
    '''
    Add the given configurations to the node.
    @param cfgs: the list with loaded configuration, which contains this node.
    @type cfgs: C{[str, ...]}
    '''
    self.cfgs = list(set(self.cfgs) | set(cfgs))

  def remConfig(self, cfgs):
    '''
    Remove the given configurations from the node.
    @param cfgs: the list with loaded configuration, which contains this node.
    @type cfgs: C{[str, ...]}
    '''
    self.cfgs = list(set(self.cfgs) - set(cfgs))

  def addDefaultConfig(self, default_cfgs):
    '''
    Add the given default configurations to the node.
    @param default_cfgs: the list with default configurations, which contains this node.
    @type default_cfgs: C{[str, ...]} 
    @see: U{http://ros.org/wiki/default_cfg_fkie|default_cfg_fkie}
    '''
    self.default_cfgs = list(set(self.default_cfgs) | set(default_cfgs))

  def remDefaultConfig(self, default_cfgs):
    '''
    Remove the given default configurations from the node.
    @param default_cfgs: the list with default configurations, which contains this node. 
    @type default_cfgs: C{[str, ...]} 
    @see: U{http://ros.org/wiki/default_cfg_fkie|default_cfg_fkie}
    '''
    self.default_cfgs = list(set(self.default_cfgs) - set(default_cfgs))
    

  def updateRunState(self, other):
    '''
    Update only the informations, which can be changed while the node is running.  
    The name, mastername, masteruri and launch configurations are not changed.
    @param other: the other instance of the L{ExtendedNodeInfo}
    @type other: L{ExtendedNodeInfo}
    '''
    self.uri = other.uri
    self.pid = other.pid
    self.published = other.published
    self.subscribed = other.subscribed
    self.services = other.services


  def updateDispayedName(self, item, show_ros_names):
    '''
    Updates the name representation of the given QStandardItem
    @param item: item which represent the URI 
    @type item: L{PySide.QtGui.QStandardItem}
    @param show_ros_names: show the as ROS names or as their description.
    @type show_ros_names: C{bool}
    '''
    if self.descr_name and not show_ros_names:
      item.setText(self.descr_name)
    else:
      item.setText(NodeItem.toHTML(self.name))
    tooltip = ''.join(['<html><body><h4>', self.name, '</h4><dl>'])
    tooltip = ''.join([tooltip, '<dt>PID: ', str(self.pid), '</dt></dl>'])
    uri = nm.nameres().getUri(host=nm.nameres().getHostname(self.uri))
    master_discovered = (not uri is None)
    local = (nm.nameres().getHostname(self.uri) == nm.nameres().getHostname(self.masteruri))
    if not self.pid is None:
      item.setIcon(QtGui.QIcon(':/icons/state_run.png'))
    elif not local and not master_discovered and not self.uri is None:
      item.setIcon(QtGui.QIcon(':/icons/state_run.png'))
      tooltip = ''.join([tooltip, '<dl><dt>(Remote nodes will not be ping, so they are always marked running)</dt></dl>'])
#    elif not master_discovered and not self.uri is None:
#      item.setIcon(QtGui.QIcon(':/icons/state_run.png'))
    elif not self.uri is None:
      item.setIcon(QtGui.QIcon(':/icons/crystal_clear_warning.png'))
      if not local and master_discovered:
        tooltip = ''.join(['<h4>', self.name, ' is not local, however the ROS master on this host is discovered, but no information about this node received!', '</h4>'])
    else:
      item.setIcon(QtGui.QIcon())

    if self.descr_type or self.descr_name or self.descr:
      tooltip = ''.join([tooltip, '<b><u>Detailed description:</u></b>'])
      if self.descr_type:
        tooltip = ''.join([tooltip, '<dl><dt><b>Sensor type:</b> ', self.descr_type, '</dt>'])
      if self.descr_name:
        tooltip = ''.join([tooltip, '<dt><b>Sensor name:</b> ', self.descr_name, '</dt></dl>'])
      if self.descr:
        try:
          from docutils import examples
          tooltip = ''.join([tooltip, examples.html_body(self.descr, input_encoding='utf8')])
        except:
          import traceback
          rospy.logwarn("Error while generate description for a node: %s", str(traceback.format_exc()))
    tooltip = ''.join([tooltip, '</dl></body></html>'])
    item.setToolTip(tooltip)

  def updateDisplayedConfig(self, item):
    '''
    Updates the configuration representation of the given QStandardItem
    @param item: item which represent the URI 
    @type item: L{PySide.QtGui.QStandardItem}
    '''
    if not item is None and isinstance(item, QtGui.QStandardItem):
      item.setText(str(''.join(['[',str(len(self.cfgs)+len(self.default_cfgs)),']'])) if len(self.cfgs)+len(self.default_cfgs) > 1 else "")
      # set tooltip
      tooltip = ''
      if len(self.cfgs)+len(self.default_cfgs) > 0:
        tooltip = '<html><body>'
        if len(self.cfgs) > 0:
          tooltip = ''.join([tooltip, '<h4>', 'Configuration files:', '</h4><dl>'])
          for c in self.cfgs:
            tooltip = ''.join([tooltip, '<dt>', c, '</dt>'])
          tooltip = ''.join([tooltip, '</dl>'])
        if len(self.default_cfgs) > 0:
          tooltip = ''.join([tooltip, '<h4>', 'Default configurations:', '</h4><dl>'])
          for (name, uri) in self.default_cfgs:
            tooltip = ''.join([tooltip, '<dt>', name[0:-11] , '</dt>']) # remove the last '/list_nodes'
          tooltip = ''.join([tooltip, '</dl>'])
        tooltip = ''.join([tooltip, '</body></html>'])
      item.setToolTip(tooltip)
      # set icons
      if len(self.cfgs) > 0 and len(self.default_cfgs) > 0:
        item.setIcon(QtGui.QIcon(':/icons/crystal_clear_launch_file_def_cfg.png'))
      elif len(self.cfgs) > 0:
        item.setIcon(QtGui.QIcon(':/icons/crystal_clear_launch_file.png'))
      elif len(self.default_cfgs) > 0:
        item.setIcon(QtGui.QIcon(':/icons/crystal_clear_default_cfg.png'))
      else:
        item.setIcon(QtGui.QIcon())

  def updateDisplayedURI(self, item):
    '''
    Updates the URI representation of the given L{PySide.QtGui.QStandardItem}
    @param item: item which represent the URI 
    @type item: L{PySide.QtGui.QStandardItem}
    '''
    if not item is None and isinstance(item, QtGui.QStandardItem):
      item.setText(str(self.uri) if not self.uri is None else "")
#      item.setIcon(QtGui.QIcon(':/icons/crystal_clear_launch_file.png'))

  def setDescription(self, descr_type, descr_name, descr):
    '''
    Sets the description of the node
    @see: L{updateTooltip()}
    @param descr_type: the type of the sensor
    @type descr_type: C{str}
    @param descr_name: the name of the sensor
    @type descr_name: C{str}
    @param descr: the description of the sensor as a U{http://docutils.sourceforge.net/rst.html|reStructuredText} 
    @type descr: C{str}
    '''
    self.descr_type = descr_type
    self.descr_name = descr_name
    self.descr = descr


class HostItem(QtGui.QStandardItem):
  '''
  The HostItem stores the information about a host. 
  '''
  ITEM_TYPE = QtCore.Qt.UserRole + 25
  
  def __init__(self, name, masteruri, local):
    '''
    Initialize the HostItem object with given values.
    @param name: the name of the host
    @type name: C{str}
    @param masteruri: the URI of ROS master located at this host
    @type masteruri: C{str}
    @param local: is this host the localhost where the node_manager is running.
    @type local: C{bool}
    '''
    QtGui.QStandardItem.__init__(self, NodeItem.toHTML(name))
    self.name = name
    self.masteruri = masteruri
    if QtCore.QFile.exists(''.join([nm.ROBOTS_DIR, name, '.png'])):
      self.setIcon(QtGui.QIcon(''.join([nm.ROBOTS_DIR, name, '.png'])))
    else:
      if local:
        self.setIcon(QtGui.QIcon(':/icons/computer.png'))
      else:
        self.setIcon(QtGui.QIcon(':/icons/remote.png'))
    self.descr_type = self.descr_name = self.descr = ''
  
  def updateTooltip(self):
    '''
    Creates a tooltip description based on text set by L{updateDescription()} 
    and all childs of this host with valid sensor description. The result is
    returned as a HTML part.
    @return: the tooltip description coded as a HTML part 
    @rtype: C{str}
    '''
    tooltip = ''
    if self.descr_type or self.descr_name or self.descr:
      tooltip = ''.join(['<h4>', self.descr_name, '</h4><dl>'])
      tooltip = ''.join([tooltip, '<dt>Type: ', self.descr_type, '</dt></dl>'])
      tooltip = ''.join([tooltip, '<b><u>Detailed description:</u></b>'])
      if self.descr:
        try:
          from docutils import examples
          tooltip = ''.join([tooltip, examples.html_body(self.descr, input_encoding='utf8')])
        except:
          import traceback
          rospy.logwarn("Error while generate description for a tooltip: %s", str(traceback.format_exc()))
    # get sensors
    sensors = []
    for j in range(self.rowCount()):
      nodeItem = self.child(j)
      if nodeItem.node.descr_name:
        sensor = nodeItem.node.descr_name
        if nodeItem.node.descr_type:
          sensor = ' '.join([sensor, '(', nodeItem.node.descr_type, ')'])
        sensors.append(sensor)
    if sensors:
      tooltip = ''.join([tooltip, '<b><u>sensors:</u></b>'])
      try:
        from docutils import examples
        tooltip = ''.join([tooltip, examples.html_body(''.join(['- ', '\n- '.join(sensors)]), input_encoding='utf8')])
      except:
        import traceback
        rospy.logwarn("Error while generate description for a tooltip: %s", str(traceback.format_exc()))
    self.setToolTip(''.join(['<html><body>', tooltip, '</body></html>']) if tooltip else self.name)
    return ''.join(['<div>', tooltip, '</div>']) if tooltip else ''
  
  def updateDescription(self, descr_type, descr_name, descr):
    '''
    Sets the description of the robot. To update the tooltip of the host item use L{updateTooltip()}.
    @param descr_type: the type of the robot
    @type descr_type: C{str}
    @param descr_name: the name of the robot
    @type descr_name: C{str}
    @param descr: the description of the robot as a U{http://docutils.sourceforge.net/rst.html|reStructuredText} 
    @type descr: C{str}
    '''
    self.descr_type = descr_type
    self.descr_name = descr_name
    self.descr = descr

  def type(self):
    return HostItem.ITEM_TYPE

  def __eq__(self, item):
    '''
    Compares the name of the host.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.name.lower() == item.lower()
    elif not (item is None):
      return self.name.lower() == item.name.lower()
    return False

  def __gt__(self, item):
    '''
    Compares the name of the host.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.name.lower() > item.lower()
    elif not (item is None):
      return self.name.lower() > item.name.lower()
    return False


class NodeItem(QtGui.QStandardItem):
  '''
  The NodeItem stores the information about the node using the ExtendedNodeInfo
  class and represents it in a L{PySide.QtGui.QTreeModel} using the 
  L{PySide.QtGui.QStandardItemModel}
  '''
  
  ITEM_TYPE = QtGui.QStandardItem.UserType + 35

  def __init__(self, ext_node):
    '''
    Initialize the NodeItem instance.
    @param ext_node: the node information
    @type ext_node: L{ExtendedNodeInfo}
    '''
    QtGui.QStandardItem.__init__(self, self.toHTML(ext_node.name))
    self.node = ext_node
    '''@ivar: the stored node information as L{ExtendedNodeInfo}'''

  def updateNodeView(self, parent, show_ros_names):
    '''
    This method is called after the self.node is changed to update the 
    representation of the node. The name will not be changed, but all other 
    data.
    @param parent: Item which contains this node item, usually the L{HostItem}. 
    This is needed to update other columns of this node.
    @type parent: L{PySide.QtGui.QStandardItem}
    @param show_ros_names: show the as ROS names or as their description.
    @type show_ros_names: C{bool}
    '''
    self.node.updateDispayedName(self, show_ros_names)
    if not parent is None:
      #update the node configurations
      child = parent.child(self.row(), 1)
      if not child is None:
        self.node.updateDisplayedConfig(child)
      #update the node uri
      child = parent.child(self.row(), 2)
      if not child is None:
        self.node.updateDisplayedURI(child)

  def type(self):
    return NodeItem.ITEM_TYPE

  @classmethod
  def getItemList(self, ext_node, show_ros_names):
    '''
    Creates the list of the items from given L{ExtendedNodeInfo}. This list is 
    used for the visualization of node data as a table row.
    @param ext_node: the node data
    @type ext_node: L{ExtendedNodeInfo}
    @param show_ros_names: show the as ROS names or as their description.
    @type show_ros_names: C{bool}
    @return: the list for the representation as a row
    @rtype: C{[L{NodeItem}, L{PySide.QtGui.QStandardItem}, ...]}
    '''
    items = []
    item = NodeItem(ext_node)
    ext_node.updateDispayedName(item, show_ros_names)
    items.append(item)
    cfgitem = QtGui.QStandardItem()
    ext_node.updateDisplayedConfig(cfgitem)
    items.append(cfgitem)
    uriitem = QtGui.QStandardItem()
    ext_node.updateDisplayedURI(uriitem)
    items.append(uriitem)
    return items

  @classmethod
  def toHTML(cls, node_name):
    '''
    Creates a HTML representation of the node name.
    @param node_name: the name of the node
    @type node_name: C{str}
    @return: the HTML representation of the name of the node
    @rtype: C{str}
    '''
    ns, sep, name = node_name.rpartition('/')
    result = ''
    if sep:
      result = ''.join(['<html><body>', '<span style="color:gray;">', str(ns), sep, '</span><b>', name, '</b></body></html>'])
    else:
      result = name
    return result

  def __eq__(self, item):
    '''
    Compares the name of the node.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.node.name.lower() == item.lower()
    elif not (item is None):
      return self.node.name.lower() == item.node.name.lower()
    return False

  def __gt__(self, item):
    '''
    Compares the name of the node.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.node.name.lower() > item.lower()
    elif not (item is None):
      return self.node.name.lower() > item.node.name.lower()
    return False

class NodeTreeModel(QtGui.QStandardItemModel):
  '''
  The model to show the nodes running in a ROS system or loaded by a launch
  configuration.
  '''
#  ICONS = {'default'        : QtGui.QIcon(),
#           'run'            : QtGui.QIcon(":/icons/state_run.png"),
#           'warning'        : QtGui.QIcon(":/icons/crystal_clear_warning.png"),
#           'def_launch_cfg' : QtGui.QIcon(":/icons/crystal_clear_launch_file_def_cfg.png"),
#           'launch_cfg'     : QtGui.QIcon(":/icons/crystal_clear_launch_file.png"),
#           'def_cfg'        : QtGui.QIcon(":/icons/crystal_clear_default_cfg.png") }
  
  header = [('Name', 300),
            ('Cfgs', 60),
            ('URI', -1)]

  hostInserted = QtCore.Signal(QtCore.QModelIndex)
  '''@ivar: the Qt signal, which is emitted, if a new host was inserted. 
  Parameter: L{QtCore.QModelIndex} of the inserted host item'''

  def __init__(self, masteruri, parent=None):
    '''
    Initialize the model.
    '''
    super(NodeTreeModel, self).__init__(parent)
    self.setColumnCount(len(NodeTreeModel.header))
    self.setHorizontalHeaderLabels([label for label, width in NodeTreeModel.header])
#    self.rootItem = NodeTreeItem([name for name, width in self.header])
    self.show_rosnames = True

  def show_ros_names(self, value):
    self.show_rosnames = value
    for i in range(self.invisibleRootItem().rowCount()):
      host = self.invisibleRootItem().child(i)
      if not host is None: # should not occur
        for j in range(host.rowCount()):
          host.child(j).updateNodeView(host, value)


  def flags(self, index):
    if not index.isValid():
      return QtCore.Qt.NoItemFlags
    return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable
  
  def getHostItem(self, mastername, masteruri, onhost=''):
    '''
    Searches for the host item in the model. If no item is found a new one will 
    created and inserted in sorted order.
    @param mastername: the name of the master
    @type mastername: C{str}
    @param masteruri: used in case of creation a new host item 
    @type masteruri: C{str}
    @param onhost: the name of the displayed host
    @type onhost: C{str}
    @return: the item associated with the given master
    @rtype: L{HostItem}
    '''
    local = (onhost == mastername)
    root = self.invisibleRootItem()
    for i in range(root.rowCount()):
      if root.child(i) == mastername:
        return root.child(i)
      elif root.child(i) > mastername:
        hostItem = HostItem(mastername, masteruri, local)
        self.insertRow(i, hostItem)
        self.hostInserted.emit(hostItem)
        return hostItem
    self.appendRow(HostItem(mastername, masteruri, local))
    self.hostInserted.emit(self.invisibleRootItem().child(self.invisibleRootItem().rowCount()-1))
    return root.child(root.rowCount()-1)

  def updateModelData(self, nodes_extended, onhost):
    '''
    Updates the model data.
    @param nodes_extended: a dictionary with name and extended info of the nodes.
    @type nodes_extended: C{dict(node name : L{ExtendedNodeInfo}, ...)}
    @param onhost: the displayed host
    @type onhost: C{str}
    '''
    #remove old nodes from the list
    nodes = nodes_extended.keys()
    for i in reversed(range(self.invisibleRootItem().rowCount())):
      host = self.invisibleRootItem().child(i)
      if not host is None: # should not occur
        for j in reversed(range(host.rowCount())):
          nodeItem = host.child(j)
          if not nodeItem.node.name in nodes:
            nodeItem.node.updateRunState(ExtendedNodeInfo(nodeItem.node.name, None, None, None, None))
            if self.canBeremoved(nodeItem.node):
              host.removeRow(j)
            else:
              nodeItem.updateNodeView(host, self.show_rosnames)
          elif nodes_extended[nodeItem.node.name].uri != nodeItem.node.uri and not nodeItem.node.cfgs and not nodeItem.node.default_cfgs:
            # if the node was started on the other host, remove the current existing
            host.removeRow(j)
      else:
        return
      if host.rowCount() == 0:
        self.invisibleRootItem().removeRow(i)
    for (name, node) in nodes_extended.items():
      # create parent items for different hosts
      host = nm.nameres().getHostname(node.uri if not node.uri is None else node.masteruri)
      if not host is None:
        hostItem = self.getHostItem(host, node.masteruri, onhost)
        doAddItem = True
        for i in range(hostItem.rowCount()):
          nodeItem = hostItem.child(i)
          if (nodeItem == node.name):
            # update item
            nodeItem.node.updateRunState(node)
            nodeItem.updateNodeView(hostItem, self.show_rosnames)
            doAddItem = False
            break
          elif (hostItem.child(i) > node.name):
            hostItem.insertRow(i, NodeItem.getItemList(node, self.show_rosnames))
            doAddItem = False
            break
        if doAddItem:
          hostItem.appendRow(NodeItem.getItemList(node, self.show_rosnames))
      else: # should not happen!
        print "node IGNORED", name, " - no host detected, uri:", node.uri, ", masteruri:", node.masteruri

  def appendConfigNodes(self, nodes_extended, onhost):
    '''
    Adds nodes to the model. If the node is already in the model, only his 
    configuration list will be extended.
    @param nodes_extended: a dictionary with nodes and his names
    @type nodes_extended: C{dict(node name : L{ExtendedNodeInfo}, ...)} 
    '''
    for (name, node) in nodes_extended.items():
      # create parent items for different hosts
      hostItem = self.getHostItem(node.mastername, node.masteruri, onhost)
      doAddItem = True
      for i in range(hostItem.rowCount()):
        nodeItem = hostItem.child(i)
        if (hostItem.child(i) == node.name):
          # update item
          hostItem.child(i).node.addConfig(node.cfgs)
          hostItem.child(i).updateNodeView(hostItem, self.show_rosnames)
          doAddItem = False
          break
        elif (hostItem.child(i) > node.name):
          hostItem.insertRow(i, NodeItem.getItemList(node, self.show_rosnames))
          doAddItem = False
          break
      if doAddItem:
        hostItem.appendRow(NodeItem.getItemList(node, self.show_rosnames))

  def removeConfigNodes(self, nodes_extended):
    '''
    Removes nodes from the model. If node is running or containing in other
    launch or default configurations , only his configuration list will be 
    reduced.
    @param nodes_extended: a dictionary with nodes and his names
    @type nodes_extended: C{dict(node name : L{ExtendedNodeInfo}, ...)} 
    '''
    nodenames= nodes_extended.keys()
    for i in reversed(range(self.invisibleRootItem().rowCount())):
      host = self.invisibleRootItem().child(i)
      if not host is None:
        for j in reversed(range(host.rowCount())):
          nodeItem = host.child(j)
          if nodeItem.node.name in nodenames:
            nodeItem.node.remConfig(nodes_extended[nodeItem.node.name].cfgs)
            if self.canBeremoved(nodeItem.node):
              host.removeRow(j)
            else:
              nodeItem.updateNodeView(host, self.show_rosnames)
        if host.rowCount() == 0:
          self.invisibleRootItem().removeRow(i)

  def appendDefaultConfigNodes(self, nodes_extended, onhost):
    '''
    Adds nodes to the model. If the node is already in the model, only his 
    C{default configuration} list will be extended.
    @param nodes_extended: a dictionary with nodes and his names
    @type nodes_extended: C{dict(node name : L{ExtendedNodeInfo}, ...)} 
    '''
    for (name, node) in nodes_extended.items():
      # create parent items for different hosts
      hostItem = self.getHostItem(node.mastername, node.masteruri, onhost)
      doAddItem = True
      for i in range(hostItem.rowCount()):
        nodeItem = hostItem.child(i)
        if (hostItem.child(i) == node.name):
          # update item
          hostItem.child(i).node.addDefaultConfig(node.default_cfgs)
          hostItem.child(i).updateNodeView(hostItem, self.show_rosnames)
          doAddItem = False
          break
        elif (hostItem.child(i) > node.name):
          hostItem.insertRow(i, NodeItem.getItemList(node, self.show_rosnames))
          doAddItem = False
          break
      if doAddItem:
        hostItem.appendRow(NodeItem.getItemList(node, self.show_rosnames))

  def removeDefaultConfigNodes(self, nodes_extended):
    '''
    Removes nodes from the model. If node is running or containing in other
    launch or default configurations , only his C{default configuration} list will be 
    reduced.
    @param nodes_extended: a dictionary with nodes and his names
    @type nodes_extended: C{dict(node name : L{ExtendedNodeInfo}, ...)} 
    '''
    nodenames= nodes_extended.keys()
    for i in reversed(range(self.invisibleRootItem().rowCount())):
      host = self.invisibleRootItem().child(i)
      if not host is None:
        for j in reversed(range(host.rowCount())):
          nodeItem = host.child(j)
          if nodeItem.node.name in nodenames:
            nodeItem.node.remDefaultConfig(nodes_extended[nodeItem.node.name].default_cfgs)
            if self.canBeremoved(nodeItem.node):
              host.removeRow(j)
            else:
              nodeItem.updateNodeView(host, self.show_rosnames)
        if host.rowCount() == 0:
          self.invisibleRootItem().removeRow(i)
        else:
          host.updateTooltip()

  def canBeremoved(self, node):
    '''
    @return: True if the node has no configuration and is not running, so the pid 
    and node URI are C{None}
    @rtype: C{bool}
    '''
    return (node.pid is None and node.uri is None and (len(node.cfgs)+len(node.default_cfgs)) == 0)

  def updateNodesDescription(self, onhost, items):
    '''
    Updates the description of the nodes received from default_cfg
    @param onhost: the host, where the node is located
    @type onhost: C{str} 
    @param items: list with descriptions
    @type items: C{[L{default_cfg_fkie.Description}]}
    @return: the host description codes a HTML or an empty string 
    @rtype: C{str}
    '''
    ROBOT_ID = 'robot'
    try:
      from default_cfg_fkie.msg import Description
      ROBOT_ID = Description.ID_ROBOT
    except:
      import traceback
      rospy.logwarn("%s", str(traceback.format_exc()))

    root = self.invisibleRootItem()
    for i in range(root.rowCount()):
      if root.child(i) == onhost:
        host = root.child(i)
        for item in items:
          if (intern(item.id) == intern(ROBOT_ID)):
            host.updateDescription(item.type, item.name, item.description)
          else:
            for j in range(host.rowCount()):
              nodeItem = host.child(j)
              if nodeItem.node.name == item.ros_name:
                nodeItem.node.setDescription(item.type, item.name, item.description)
                nodeItem.updateNodeView(host, self.show_rosnames)
        return host.updateTooltip()
    return ''

  def updateNodesDescr(self, onhost, nodes):
    '''
    Updates the description of the nodes
    @param onhost: the host, where the node is located
    @type onhost: C{str} 
    @param nodes: dictonary with descriptions
    @type nodes: C{dict(sensor:dict({'sensor_type', 'sensor_name', 'sensor_descr'}:str(value)))}
    @return: the host description codes a HTML or an empty string 
    @rtype: C{str}
    '''
    root = self.invisibleRootItem()
    for i in range(root.rowCount()):
      if root.child(i) == onhost:
        host = root.child(i)
        for node, d in nodes.items():
          for j in range(host.rowCount()):
            nodeItem = host.child(j)
            if nodeItem.node.name == node:
              nodeItem.node.setDescription(d['sensor_type'], d['sensor_name'], d['sensor_descr'])
              nodeItem.updateNodeView(host, self.show_rosnames)
        return host.updateTooltip()
    return ''

  def updateHostDescription(self, host, descr_type, descr_name, descr):
    '''
    Updates the description of a host.
    @param host: host to update
    @type host: C{str}
    @param descr_type: the type of the robot
    @type descr_type: C{str}
    @param descr_name: the name of the robot
    @type descr_name: C{str}
    @param descr: the description of the robot as a U{http://docutils.sourceforge.net/rst.html|reStructuredText} 
    @type descr: C{str}
    '''
    root = self.invisibleRootItem()
    for i in range(root.rowCount()):
      if root.child(i) == host:
        h = root.child(i)
        h.updateDescription(descr_type, descr_name, descr)
        return h.updateTooltip()
