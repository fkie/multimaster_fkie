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

from PySide import QtCore
from PySide import QtGui

import roslib
import roslib.message
import roslib.msgs

class TopicItem(QtGui.QStandardItem):
  '''
  The topic item stored in the topic model. This class stores the topic as
  L{master_discovery_fkie.TopicInfo}. The name of the topic represented in HTML.
  '''
  
  ITEM_TYPE = QtGui.QStandardItem.UserType + 36

  def __init__(self, topic, parent=None):
    '''
    Initialize the topic item.
    @param topic: the topic object to view
    @type topic: L{master_discovery_fkie.TopicInfo}
    '''
    QtGui.QStandardItem.__init__(self, self.toHTML(topic.name))
    self.topic = topic
    '''@ivar: service info as L{master_discovery_fkie.ServiceInfo}.'''

  def updateTopicView(self, parent):
    '''
    This method is called after the self.topic is changed to update the 
    representation of the topic. The name will not be changed, but all other 
    data.
    @param parent: Item which contains this topic item. This is needed to update 
    other columns of this topic.
    @type parent: L{PySide.QtGui.QStandardItem}
    '''
    if not parent is None:
      #update the topic publishers
      child = parent.child(self.row(), 1)
      if not child is None:
        self.updatePublisherView(self.topic, child)
      #update the topic subscriber
      child = parent.child(self.row(), 2)
      if not child is None:
        self.updateSubscriberView(self.topic, child)
      #update the topic type
      child = parent.child(self.row(), 3)
      if not child is None:
        self.updateTypeView(self.topic, child)

  @classmethod
  def updatePublisherView(cls, topic, item):
    '''
    Updates the representation of the column contains the publisher state.
    @param topic: the topic data
    @type topic: L{master_discovery_fkie.TopicInfo}
    @param item: corresponding item in the model
    @type item: L{TopicItem}
    '''
    item.setText(str(len(topic.publisherNodes)))
    tooltip = ''.join(['<h4>', 'Publisher [', topic.name, ']:</h4><dl>'])
    for p in topic.publisherNodes:
      tooltip = ''.join([tooltip, '<dt>', p, '</dt>'])
    tooltip = ''.join([tooltip, '</dl>'])
    if len(topic.publisherNodes) > 0:
      item.setToolTip(''.join(['<div>', tooltip, '</div>']))

  @classmethod
  def updateSubscriberView(cls, topic, item):
    '''
    Updates the representation of the column contains the subscriber state.
    @param topic: the topic data
    @type topic: L{master_discovery_fkie.TopicInfo}
    @param item: corresponding item in the model
    @type item: L{TopicItem}
    '''
    item.setText(str(len(topic.subscriberNodes)))
    tooltip = ''.join(['<h4>', 'Subscriber [', topic.name, ']:</h4><dl>'])
    for p in topic.subscriberNodes:
      tooltip = ''.join([tooltip, '<dt>', p, '</dt>'])
    tooltip = ''.join([tooltip, '</dl>'])
    if len(topic.subscriberNodes) > 0:
      item.setToolTip(''.join(['<div>', tooltip, '</div>']))

  @classmethod
  def updateTypeView(cls, topic, item):
    '''
    Updates the representation of the column contains the type of the topic.
    @param topic: the topic data
    @type topic: L{master_discovery_fkie.TopicInfo}
    @param item: corresponding item in the model
    @type item: L{TopicItem}
    '''
    item.setText(str(topic.type))
    if not topic.type is None:
      tooltip = ''
      try:
        mclass = roslib.message.get_message_class(topic.type)
        tooltip = str(mclass)
        if not mclass is None:
          tooltip = str(mclass.__slots__)
          for f in mclass.__slots__:
            idx = mclass.__slots__.index(f)
            idtype = mclass._slot_types[idx]
            base_type = roslib.msgs.base_msg_type(idtype)
            primitive = "unknown"
            if base_type in roslib.msgs.PRIMITIVE_TYPES:
              primitive = "primitive"
            else:
              try:
                list_msg_class =roslib.message.get_message_class(base_type)
                primitive = "class", list_msg_class.__slots__
              except ValueError:
                pass
            tooltip = ''.join([tooltip, '\n\t', str(f), ': ', str(idtype), ' (', str(primitive),')'])
      except ValueError:
        pass
      item.setToolTip(tooltip)
  
  @classmethod
  def toHTML(cls, topic_name):
    '''
    Creates a HTML representation of the topic name.
    @param topic_name: the topic name
    @type topic_name: C{str}
    @return: the HTML representation of the topic name
    @rtype: C{str}
    '''
    ns, sep, name = topic_name.rpartition('/')
    result = ''
    if sep:
      result = ''.join(['<div>', '<span style="color:gray;">', str(ns), sep, '</span><b>', name, '</b></div>'])
    else:
      result = name
    return result

  def type(self):
    return TopicItem.ITEM_TYPE

  @classmethod
  def getItemList(self, topic):
    '''
    Creates the list of the items from topic. This list is used for the 
    visualization of topic data as a table row.
    @param topic: the topic data
    @type topic: L{master_discovery_fkie.TopicInfo}
    @return: the list for the representation as a row
    @rtype: C{[L{TopicItem} or L{PySide.QtGui.QStandardItem}, ...]}
    '''
    items = []
    item = TopicItem(topic)
    items.append(item)
    pubItem = QtGui.QStandardItem()
    TopicItem.updatePublisherView(topic, pubItem)
    items.append(pubItem)
    subItem = QtGui.QStandardItem()
    TopicItem.updateSubscriberView(topic, subItem)
    items.append(subItem)
    typeItem = QtGui.QStandardItem()
    TopicItem.updateTypeView(topic, typeItem)
    items.append(typeItem)
    return items



  def __eq__(self, item):
    '''
    Compares the name of topic.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.topic.name.lower() == item.lower()
    elif not (item is None):
      return self.topic.name.lower() == item.topic.name.lower()
    return False

  def __gt__(self, item):
    '''
    Compares the name of topic.
    '''
    if isinstance(item, str) or isinstance(item, unicode):
      return self.topic.name.lower() > item.lower()
    elif not (item is None):
      return self.topic.name.lower() > item.topic.name.lower()
    return False


class TopicModel(QtGui.QStandardItemModel):
  '''
  The model to manage the list with topics in ROS network.
  '''
  header = [('Name', 300),
            ('Publisher', 50), 
            ('Subscriber', 50),
            ('Type', -1)]
  '''@ivar: the list with columns C{[(name, width), ...]}'''
  
  def __init__(self):
    '''
    Creates a new list model.
    '''
    QtGui.QStandardItemModel.__init__(self)
    self.setColumnCount(len(TopicModel.header))
    self.setHorizontalHeaderLabels([label for label, width in TopicModel.header])

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
    return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable

  def updateModelData(self, topics):
    '''
    Updates the topics model. New topic will be inserted in sorting order. Not 
    available topics removed from the model.
    @param topics: The dictionary with topics 
    @type topics: C{dict(topic name : L{master_discovery_fkie.TopicInfo}, ...)}
    '''
    topic_names = topics.keys()
    root = self.invisibleRootItem()
    for i in reversed(range(root.rowCount())):
      topicItem = root.child(i)
      if not topicItem.topic.name in topic_names:
        root.removeRow(i)
    for (name, topic) in topics.items():
      doAddItem = True
      for i in range(root.rowCount()):
        topicItem = root.child(i)
        if (topicItem == topic.name):
          # update item
          topicItem.topic = topic
          topicItem.updateTopicView(root)
          doAddItem = False
          break
        elif (topicItem > topic.name):
          root.insertRow(i, TopicItem.getItemList(topic))
          doAddItem = False
          break
      if doAddItem:
        root.appendRow(TopicItem.getItemList(topic))

