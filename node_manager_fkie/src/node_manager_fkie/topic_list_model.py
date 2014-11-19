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

from python_qt_binding import QtCore
from python_qt_binding import QtGui


from master_discovery_fkie.master_info import TopicInfo
from detailed_msg_box import WarningMessageBox

class TopicItem(QtGui.QStandardItem):
  '''
  The topic item stored in the topic model. This class stores the topic as
  L{master_discovery_fkie.TopicInfo}. The name of the topic represented in HTML.
  '''
  
  ITEM_TYPE = QtGui.QStandardItem.UserType + 36
  NAME_ROLE = QtCore.Qt.UserRole + 1
  NODENAMES_ROLE = QtCore.Qt.UserRole + 2
  COL_PUB = 1
  COL_SUB = 2
  COL_TYPE = 3


  def __init__(self, name, topic=None, parent=None):
    '''
    Initialize the topic item.
    @param name: the topic name
    @type name: C{str}
    '''
    QtGui.QStandardItem.__init__(self, name)
    self.parent_item = parent
    '''@ivar: service info as L{master_discovery_fkie.ServiceInfo}.'''
    self._publish_thread = None
    self.topic = TopicInfo(name) if topic is None else topic

#  def __del__(self):
#    print "delete TOPIC", self.__topic.name

  def updateView(self):
    '''
    Updates the view
    '''
    self.updatePublisherView()
    self.updateSubscriberView()
    self.updateTypeView()

  def updatePublisherView(self):
    '''
    Updates the representation of the column contains the publisher state.
    '''
    if not self.parent_item is None:
      cfg_col = self.parent_item.child(self.row(), TopicItem.COL_PUB)
      if not cfg_col is None and isinstance(cfg_col, QtGui.QStandardItem):
        cfg_col.setText(str(len(self.topic.publisherNodes)))
        tooltip = ''.join(['<h4>', 'Publisher [', self.topic.name, ']:</h4><dl>'])
        for p in self.topic.publisherNodes:
          tooltip = ''.join([tooltip, '<dt>', p, '</dt>'])
        tooltip = ''.join([tooltip, '</dl>'])
        if len(self.topic.publisherNodes) > 0:
          cfg_col.setToolTip(''.join(['<div>', tooltip, '</div>']))

  def updateSubscriberView(self):
    '''
    Updates the representation of the column contains the subscriber state.
    '''
    if not self.parent_item is None:
      cfg_col = self.parent_item.child(self.row(), TopicItem.COL_SUB)
      if not cfg_col is None and isinstance(cfg_col, QtGui.QStandardItem):
        cfg_col.setText(str(len(self.topic.subscriberNodes)))
        tooltip = ''.join(['<h4>', 'Subscriber [', self.topic.name, ']:</h4><dl>'])
        for p in self.topic.subscriberNodes:
          tooltip = ''.join([tooltip, '<dt>', p, '</dt>'])
        tooltip = ''.join([tooltip, '</dl>'])
        if len(self.topic.subscriberNodes) > 0:
          cfg_col.setToolTip(''.join(['<div>', tooltip, '</div>']))

  def updateTypeView(self):
    '''
    Updates the representation of the column contains the type of the topic.
    '''
    if not self.parent_item is None:
      cfg_col = self.parent_item.child(self.row(), TopicItem.COL_TYPE)
      if not cfg_col is None and isinstance(cfg_col, QtGui.QStandardItem):
        cfg_col.setText(self.topic.type if self.topic.type and self.topic.type != 'None' else 'unknown type')
        # removed tooltip for clarity!!!
#         if not self.topic.type is None and not cfg_col.toolTip():
#           return
#           # removed tooltip for clarity !!!
# #          tooltip = ''
#           try:
#             mclass = roslib.message.get_message_class(self.topic.type)
# #            tooltip = str(mclass)
#             if not mclass is None:
# #              tooltip = str(mclass.__slots__)
#               for f in mclass.__slots__:
#                 idx = mclass.__slots__.index(f)
#                 idtype = mclass._slot_types[idx]
#                 base_type = roslib.msgs.base_msg_type(idtype)
#                 primitive = "unknown"
#                 if base_type in roslib.msgs.PRIMITIVE_TYPES:
#                   primitive = "primitive"
#                 else:
#                   try:
#                     list_msg_class = roslib.message.get_message_class(base_type)
#                     primitive = "class", list_msg_class.__slots__
#                   except ValueError:
#                     pass
# #                tooltip = ''.join([tooltip, '\n\t', str(f), ': ', str(idtype), ' (', str(primitive),')'])
#           except ValueError:
#             pass
#          cfg_col.setToolTip(tooltip)

  def _on_wait_for_publishing(self):
    self.updateIconView(QtGui.QIcon(':/icons/state_off.png'))

  def _on_partial_publishing(self):
    self.updateIconView(QtGui.QIcon(':/icons/state_part.png'))

  def _on_publishing(self):
    self.updateIconView(QtGui.QIcon(':/icons/state_run.png'))
  
  def _publish_finished(self):
    self._publish_thread = None
    self.setIcon(QtGui.QIcon())
  
  def show_error_msg(self, msg):
    WarningMessageBox(QtGui.QMessageBox.Warning, "Publish error", 
                  'Error while publish to %s'%self.topic.name,
                  str(msg)).exec_()

  def type(self):
    return TopicItem.ITEM_TYPE

  def data(self, role):
    if role == self.NAME_ROLE:
      return self.topic.name
    elif role == self.NODENAMES_ROLE:
      return str(self.topic.publisherNodes)+str(self.topic.subscriberNodes)
    else:
      return QtGui.QStandardItem.data(self, role)

  @classmethod
  def getItemList(self, topic, root):
    '''
    Creates the list of the items from topic. This list is used for the 
    visualization of topic data as a table row.
    @param name: the topic name
    @type name: C{str}
    @param root: The parent QStandardItem
    @type root: L{PySide.QtGui.QStandardItem}
    @return: the list for the representation as a row
    @rtype: C{[L{TopicItem} or L{PySide.QtGui.QStandardItem}, ...]}
    '''
    items = []
    item = TopicItem(topic.name, topic, parent=root)
    items.append(item)
    pubItem = QtGui.QStandardItem()
#    TopicItem.updatePublisherView(topic, pubItem)
    items.append(pubItem)
    subItem = QtGui.QStandardItem()
#    TopicItem.updateSubscriberView(topic, subItem)
    items.append(subItem)
    typeItem = QtGui.QStandardItem()
#    TopicItem.updateTypeView(topic, typeItem)
    items.append(typeItem)
    return items


#  def __eq__(self, item):
#    '''
#    Compares the name of topic.
#    '''
#    if isinstance(item, str) or isinstance(item, unicode):
#      return self.topic.name.lower() == item.lower()
#    elif not (item is None):
#      return self.topic.name.lower() == item.topic.name.lower()
#    return False
#
#  def __gt__(self, item):
#    '''
#    Compares the name of topic.
#    '''
#    if isinstance(item, str) or isinstance(item, unicode):
#      return self.topic.name.lower() > item.lower()
#    elif not (item is None):
#      return self.topic.name.lower() > item.topic.name.lower()
#    return False


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
    self.setHorizontalHeaderLabels([label for label, _ in TopicModel.header])
    self.pyqt_workaround = dict() # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass

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

  def updateModelData(self, topics, added_topics, updated_topics, removed_topics):
    '''
    Updates the topics model. New topic will be inserted in sorting order. Not 
    available topics removed from the model.
    @param topics: The dictionary with topics 
    @type topics: C{dict(topic name : L{master_discovery_fkie.TopicInfo}, ...)}
    @param added_topics: the list of new topics in the :topics: list
    @type added_topics: list or set
    @param updated_topics: the list of updated topics in the :topics: list
    @type updated_topics: list or set
    @param removed_topics: the list of removed topics in the :topics: list
    @type removed_topics: list or set
    '''
    root = self.invisibleRootItem()
    #remove or update the existing items
    for i in reversed(range(root.rowCount())):
      topicItem = root.child(i)
      if topicItem.topic.name in removed_topics:
        root.removeRow(i)
        try:
          del self.pyqt_workaround[topicItem.topic.name] # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
        except:
          pass
      elif topicItem.topic.name in updated_topics:
        topicItem.updateView()
#    cputimes = os.times()
#    cputime_init = cputimes[0] + cputimes[1]
    # insert other items in sorted order
    for topic_name in added_topics:
      try:
        doAddItem = True
        topic = topics[topic_name]
        for i in range(root.rowCount()):
          if not topic_name in updated_topics:
            topicItem = root.child(i)
            if cmp(topicItem.topic.name, topic_name) > 0:
              new_item_row = TopicItem.getItemList(topic, root)
              root.insertRow(i, new_item_row)
              self.pyqt_workaround[topic_name] = new_item_row[0] # workaround for using with PyQt: store the python object to keep the defined attributes in the TopicItem subclass
              new_item_row[0].updateView()
              doAddItem = False
              break
          else:
            doAddItem = False
            break
        if doAddItem:
          new_item_row = TopicItem.getItemList(topic, root)
          root.appendRow(new_item_row)
          self.pyqt_workaround[topic_name] = new_item_row[0]
          new_item_row[0].updateView()
      except:
        pass
#    cputimes = os.times()
#    cputime = cputimes[0] + cputimes[1] - cputime_init
#    print "      update topic ", cputime, ", topic count:", len(topics)

  def index_from_names(self, publisher, subscriber):
    '''
    Returns for given topics the list of QModelIndex in this model.
    :param publisher: the list of publisher topics
    :type publisher: [str, ...]
    :param subscriber: the list of subscriber topics
    :type subscriber: [str, ...]
    :return: the list of QModelIndex
    :rtype: [QtCore.QModelIndex, ...]
    '''
    result = []
    root = self.invisibleRootItem()
    for i in range(root.rowCount()):
      topicItem = root.child(i)
      if topicItem.topic.name in publisher:
        result.append(self.index(i, 0))
        result.append(self.index(i, 1)) # select also the publishers column
      if topicItem.topic.name in subscriber:
        result.append(self.index(i, 0))
        result.append(self.index(i, 2)) # select also the subscribers column
    return result