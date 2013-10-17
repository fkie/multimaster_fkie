^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package node_manager_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2013-10-17)
------------------
* node_manager_fkie: fixed start button description
* node_manager_fkie: added an info button
* node_manager_fkie: changed calling of sync dialog
* node_manager_fkie: showing duplicate nodes fixed
* multimaster_fkie: fixed problems with resolving service types while sync
  while synchronization not all topics and services can be synchronized
  because of filter or errors. A detection for this case was added.
* node_manager_fkie: added user selection for remote hosts
* node_manager_fkie: fixed some paths
* node_manager_fkie: added SAVE and LOAD buttons to parameter dialog
* node_manager_fkie: fixed start nodes in multimaster on the same host
* node_manager_fkie: replaced the sync checkbox in masterlist by a sync icon
* node_manager_fkie: fixed filtering topics, services and parameter
* node_manager_fkie: buttons resized
* node_manager_fkie: added missed start parameter to master_sync
* node_manager_fkie: removed some unneeded borders in gui
* node_manager_fkie: fix loading launch file
* node_manager_fkie: fixed parameter groups
* node_manager_fkie: added new interface of dynamic_reconfigure
* node_manager_fkie: show node_manager window maximized, if the screen is small
* node_manager_fkie: fixed raise conditions
* node_manager_fkie: added filter to selected dialog and changed selection behavior
* node_manager_fkie: fix node matching
* node_manager_fkie: fixed absolute path in env of the launch file

0.3.6 (2013-09-17)
------------------
* node_manager_fkie: added a notifiaction, if `use_sim_time` parameter is set to true
* node_manager_fkie: added some control elements to node/host description
* node_manager_fkie: fix load launch file
* node_manager_fkie: fix filter in paramter dialog
* node_manager_fkie: fixed do not store the launch file on error
* node_manager_fkie: the minimum size of the parameter dialog increased
* node_manager_fkie: update the capability group of the node using the ROS parameter server, if no launch file is loaded
* node_manager_fkie: fixed cancel loading of the launch file, on cancel input args
  node_manager_fkie: do not restart anonymous nodes on relaod launch file
  node_manager_fkie: fixed closing of the remote default configs on same host but other roscore
* node_manager_fkie: resize the node_manager window on small
* node_manager_fkie: changed the intepretation of the group description
* node_manager_fkie: remove not existing remote node information. In case of restarting a ROS node without stopn a running node.
* node_manager_fkie: fixed buttons description
* node_manager_fkie: fixed change detection in included files
* node_manager_fkie: add detection of changes in the reloaded launch file and restart affected nodes
* node_manager_fkie: fixed clear_params

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
