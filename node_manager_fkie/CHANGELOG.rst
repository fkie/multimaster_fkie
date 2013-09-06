^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package node_manager_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.5 (2013-09-06)
------------------
* node_manager_fkie: fixed launch selection for favirites with same launch file name
* node_manager_fkie: fixed process id view of nodes for multiple sync hosts

0.3.4 (2013-09-05)
------------------
* node_manager_fkie: fixed file paths (removed warnings in file_watcher)
* node_manager_fkie: clear cached package names on refreshing launch file view
* node_manager_fkie: capability_group parameter can now be defined in a namespace
* node_manager_fkie: fixed pakage_name result
  added caching for package_name results

0.3.3 (2013-09-04)
------------------
* node_manager_fkie: Parse package.xml for name
  Although package folders should have the same name as the
  package, some packages (e.g. swig-wx) violate this.
  Thus, we use catkin_pkg.package.parse_package to parse
  the package.xml and look for the <name> tag, which
  contains the correct package name.
* node_manager_fkie: Install data files without executable bit
* node_manager_fkie: added a button to hide the dock widgets
* node_manager_fkie: added a question dialog to start the synchronization with a loaded config, if any exists
* node_manager_fkie: increased timeout for transfer of parameter while start of nodes
* node_manager_fkie: fixed node name creation for publishing of topics
* node_manager_fkie: fixed start of master_sync with interface file
* node_manager_fkie: removed some exeption for pyqt workaround
* node_manager_fkie: added a warning in paramter dialog
* node_manager_fkie: fixed names, preselect all files to reload after a file was changed
* node_manager_fkie: added a buttons to save and load configurations
* node_manager_fkie: show the parent of the src-folder
* node_manager_fkie: plugin renamed
* node_manager_fkie: fixed finish function to stop the running timer
* node_manager_fkie: file watcher updated, changes now notified once for all master
* multimaster_fkie: .gitignore changed
* node_manager_fkie: don't ask for argv's while reloading
* node_manager_fkie: fixed a problem while launching a default cfg nodes
* node_manager_fkie: searching for packages in rundialog after dialog opened
* node_manager_fkie: fixed waiting for roscore
* node_manager_fkie: added the default group for system nodes, fixed an often update problem
* node_manager_fkie: fixed problem while openning an editor
* node_manager_fkie: increased the wait for ROS Master
* node_manager_fkie: added the possibility to enter a varible count of list entries while calling a service or publishing to a topic
* node_manager_fkie: changed the handling while close multiple configurations
* node_manager_fkie: added the parameter as pkg:// URL to launch a default_cfg at start of node_manager
* multimaster_fkie: (*) added additional filtered interface to master_discovery rpc-server to get a filtered MasterInfo and reduce the load on network.
  (*) added the possibility to sync remote nodes using ~sync_remote_nodes parameter
* node_manager_fkie: added a possibility to create a new files
* node_manager_fkie: fixed error while browsing in launch files
* node_manager_fkie: (1) added a button to transfer launch files to remote machines,
  (2) upgraded the editor for sync dialog
  (3) added more info to progress bars
* node_manager_fkie: limited displaying frequency for echo dialog
* node_manager_fkie: limited the displayed messages in echo widget
