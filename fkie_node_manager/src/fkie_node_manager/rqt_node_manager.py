

from qt_gui.plugin import Plugin

import fkie_node_manager
from fkie_node_manager_daemon.common import utf8
from fkie_node_manager.detailed_msg_box import MessageBox

from .main_window import MainWindow


# from python_qt_binding import loadUi
class NodeManager(Plugin):

    def __init__(self, context):
        super(NodeManager, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('NodeManagerFKIE')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)
        fkie_node_manager.init_settings()
        masteruri = fkie_node_manager.settings().masteruri()
        fkie_node_manager.init_globals(masteruri)
        # Create QWidget
        try:
            self._widget = MainWindow()
#          self._widget.read_view_history()
        except Exception as e:
            MessageBox.critical(None, "Node Manager", utf8(e))
            raise
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
#        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
#        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('NodeManagerFKIEPlugin')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        fkie_node_manager.finish()
        if self._widget:
            self._widget.finish()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog
