Shortcuts
---------

**Main**

===================== ========
**F5**                Updates the status of the current host.
**Alt+N**             Opens a dialog to run a ROS node without a configuration. This node will be launched in a `SCREEN`.
**Ctrl+R**            Opens a dialog to launch a master_discovery_ node on entered host.
**Ctrl+T**            Opens a terminal and connects to the selected host using SSH.
===================== ========

**Launch-Dialog**

================================ ========
**F2**                           Renames a selected launch file.
**F4** / **Ctrl+E**              Opens an XML Editor for selected launch file.
**Ctrl+L**                       Loads selected launch file.
**Del** on history file          Removes history file from history list (Does not delete the file).
**Shift+Del**                    Deletes selected file or directory.
**Shift+LClick** on history file Goto package
**Shift+LClick** other location  Goto root
================================ ========

**Nodes-View**

===================== ========
**F3**                Opens a screen for selected node.
**F4**                Opens an XML Editor **or** parameter configuration dialog for selected node.
**Alt+R**             Runs selected nodes. Ignores already running nodes.
**Alt+S**             Stops selected nodes. If more then one node is selected, nodes ending with `rosout`, `node_manager`, `master_discovery`, `master_sync` or default_cfg are ignored.
**Ctrl+Backspace**    Sends a `SIGKILL` to selected nodes. If more then one node is selected, nodes ending with `rosout`, `node_manager`, `master_discovery`, `master_sync` and default_cfg are ignored.
**Ctrl+Delete**       Unregister selected nodes (their topics and services) from ROS master. If more then one node is selected, nodes ending with `rosout`, `node_manager`, `master_discovery`, `master_sync` and default_cfg are ignored.
**Shift+Backspace**   Sends a `SIGKILL` to assigned `SCREEN` of selected nodes.
**Shift+S**           Shows all available `SCREEN's` which contains the ROS nodes launched by node manager.
**Alt+E**             Expands all groups in Nodes tab.
**Alt+C**             Collapses all groups in Nodes tab.
**Alt+{1..5}**        Selects all nodes of the *{first...fifth}* host in Nodes tab excepting the manage nodes.
**Ctrl+X**            Copies the alternative values to clipboard. On `Node` view this is PID. On `Topic` and `Service` view it is the type. On `Parameter` view it is the parameter value.
===================== ========

**Topic/Service/Parameter-View**

===================== ========
**Ctrl+X**            Copies the alternative values to clipboard. On `Topic` and `Service` view it is the type. On `Parameter` view it is the parameter value.
===================== ========

**Editor**

===================== ========
**Ctrl+E**            Open/Close include graph.
**Ctrl+F**            Open/Close search dialog.
**Ctrl+G**            Opens goto line dialog.
**Ctrl+R**            Open/Close search/replace dialog.
**Ctrl+S**            Saves file.
**Ctrl+Shift+D**      Opens dialog with multimaster specific parameter.
**Ctrl+Shift+N**      Opens dialog to select and insert a new node.
**Ctrl+Shift+P**      Adds a new parameter at current cursor position.
**Ctrl+LClick**       Opens the included file.
===================== ========

.. _master_discovery: http://www.ros.org/wiki/master_discovery_fkie