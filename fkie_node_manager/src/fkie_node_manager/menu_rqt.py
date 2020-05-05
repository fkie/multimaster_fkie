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



from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QKeySequence
try:
    from python_qt_binding.QtGui import QAction, QMenu
except Exception:
    from python_qt_binding.QtWidgets import QAction, QMenu
from python_qt_binding.QtGui import QIcon
import fkie_node_manager as nm
import roslib


class MenuRqt(QMenu):
    '''
    This creates a menu to start a several rqt plugins.
    '''
    start_rqt_plugin_signal = Signal(str, str)
    '''
  The start_rqt_plugin_signal is emitted to start a rqt plugin (Name, Plugin).
  The Plugin can be empty, in this case the RQT itself will be start.
  '''

    def __init__(self, menu_button):
        QMenu.__init__(self)
        self.button = menu_button
        try:
            rqt_icon_path = roslib.packages.find_resource('rqt_gui', 'rqt.png').pop()
            menu_button.setText('')
            menu_button.setIcon(QIcon(rqt_icon_path))
            # creates a default config menu
            self.action_rqt_console = QAction(QIcon.fromTheme('mail-message-new'),
                                              "&Console", self,
                                              statusTip='"<p>Starts a python GUI plugin for displaying and filtering '
                                              'ROS log messages that is connected to the selected master.</p>"',
                                              triggered=self.on_show_console_clicked)
            self.addAction(self.action_rqt_console)
            self.action_rqt_logger_level = QAction(QIcon.fromTheme('format-indent-more'),
                                                   "&Logger Level", self,
                                                   statusTip='"<p>Starts a python GUI plugin for configuring the level of '
                                                   'ROS loggers that is connected to the selected master.</p>"',
                                                   triggered=self.on_show_logger_level_clicked)
            self.addAction(self.action_rqt_logger_level)
            self.action_rqt_tf_tree = QAction(QIcon.fromTheme('preferences-system-network'),
                                              "&TF Tree", self,
                                              statusTip='"<p>Starts a python GUI plugin for visualizing the TF tree'
                                              'that is connected to the selected master.</p>"',
                                              triggered=self.on_show_tf_tree_clicked)
            self.addAction(self.action_rqt_tf_tree)
            self.action_rqt_ros_graph = QAction(nm.settings().icon('button_graph.png'),
                                                "Ros &Graph", self,
                                                statusTip='"<p>Starts a python GUI plugin for visualizing the ROS computation graph'
                                                'that is connected to the selected master</p>"',
                                                triggered=self.on_show_ros_graph_clicked)
            self.addAction(self.action_rqt_ros_graph)
            self.action_rosbag_record = QAction(QIcon.fromTheme('media-record'),
                                                "rosbag record", self,
                                                statusTip='"<p>Starts the rosbag record with selected topics</p>"',
                                                triggered=self.on_start_rosbag_clicked)
            self.addAction(self.action_rosbag_record)
            self.action_rqt_rviz = QAction(QIcon.fromTheme('image-x-generic'),
                                           "R&Viz", self,
                                           statusTip='"<p>Starts RViz</p>"',
                                           triggered=self.on_show_rviz_clicked)
            self.addAction(self.action_rqt_rviz)
            self.addSeparator()
            self.action_rqt = QAction(QIcon(rqt_icon_path),
                                      "&Rqt GUI", self,
                                      statusTip='"<p>Start the rqt GUI'
                                      'that is connected to the selected master</p>"',
                                      triggered=self.on_start_rqt_clicked)
            self.addAction(self.action_rqt)
            self.action_terminal = QAction(nm.settings().icon('crystal_clear_show_io.png'),
                                           "&Terminal", self,
                                           statusTip='"<p>Start terminal on selected host</p>"',
                                           triggered=self.on_start_terminal)
            self.action_terminal.setShortcut(QKeySequence("Ctrl+T"))
            self.addAction(self.action_terminal)
            menu_button.setMenu(self)
        except Exception as _e:
            import traceback
            print(traceback.format_exc())
            menu_button.setEnabled(False)
            menu_button.setToolTip('rqt_gui not found! Please install rqt to use its plugins!')

    def on_show_console_clicked(self):
        self.start_rqt_plugin_signal.emit('Console', 'rqt_console.console.Console')

    def on_show_logger_level_clicked(self):
        self.start_rqt_plugin_signal.emit('Logger Level', 'rqt_logger_level.logger_level.LoggerLevel')

    def on_show_tf_tree_clicked(self):
        self.start_rqt_plugin_signal.emit('TF Tree', 'rqt_tf_tree.tf_tree.RosTfTree')

    def on_show_ros_graph_clicked(self):
        self.start_rqt_plugin_signal.emit('Ros Graph', 'rqt_graph.ros_graph.RosGraph')

    def on_start_rosbag_clicked(self):
        self.start_rqt_plugin_signal.emit('rosbag record', '')

    def on_show_rviz_clicked(self):
        self.start_rqt_plugin_signal.emit('RViz', 'rqt_rviz/RViz')

    def on_start_rqt_clicked(self):
        self.start_rqt_plugin_signal.emit('Rqt GUI', '')

    def on_start_terminal(self):
        self.start_rqt_plugin_signal.emit('Terminal', '')
