
Node Manager
============

A GUI to manage the configuration on local and remote ROS masters. For more details see manual_, roswiki_ or use the **(?)** button in the title bar.

.. _manual: http://fkie.github.io/multimaster_fkie/
.. _roswiki: http://www.ros.org/wiki/node_manager_fkie


Shortcuts
---------

===================== ========
**F2**                Renames a selected launch file.
**F3**                Opens a screen for selected node.
**F4**                Opens an XML Editor for selected node.
**Ctrl+R**            Opens a dialog to launch a master_discovery_ node on entered host.
**Ctrl+E**, **F4**    Opens an XML Editor for selected launch file.
**Ctrl+L**            Loads the selected launch file into selected host.
**Ctrl+O**            Runs on selected host default_cfg_ node with selected launch file as parameter.
**Alt+N**             Opens a dialog to run a ROS node without a configuration. This node will be launched in a `SCREEN`.
**Alt+R**             Runs selected nodes. Ignores already running nodes.
**Alt+S**             Stops selected nodes. If more then one node is selected, nodes ending with `rosout`, `node_manager`, `master_discovery`, `master_sync` or `default_cfg` are ignored.
**Ctrl+Backspace**    Sends a `SIGKILL` to selected nodes. If more then one node is selected, nodes ending with `rosout`, `node_manager`, `master_discovery`, `master_sync` and `default_cfg` are ignored.
**Ctrl+Delete**       Unregister selected nodes (their topics and services) from ROS master. If more then one node is selected, nodes ending with `rosout`, `node_manager`, `master_discovery`, `master_sync` and `default_cfg` are ignored.
**Shift+Backspace**   Sends a `SIGKILL` to assigned `SCREEN` of selected nodes.
**Shift+S**           Shows all available `SCREEN's` which contains the ROS nodes launched by node manager.
**Ctrl+F4**           Closes the loaded configurations.
**Alt+E**             Expands all groups in Nodes tab.
**Alt+C**             Collapses all groups in Nodes tab.
**Alt+{1..5}**        Selects all nodes of the *{first...fifth}* host in Nodes tab excepting the manage nodes.
**Ctrl+X**            Copies the alternative values to clipboard. On `Node` view this is PID. On `Topic` and `Service` view it is the type. On `Parameter` view it is the parameter value.
===================== ========

TODO
====
... write local help


.. _master_discovery: http://www.ros.org/wiki/master_discovery_fkie
.. _default_cfg: http://www.ros.org/wiki/default_cfg_fkie
