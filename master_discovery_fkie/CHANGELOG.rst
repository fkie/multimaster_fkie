^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package master_discovery_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.17 (2015-01-22)
-------------------
* master_discovery_fkie: fixed discovery support for ipv6
* Contributors: Alexander Tiderko

0.3.16 (2014-12-08)
-------------------

0.3.15 (2014-12-01)
-------------------
* multimaster_fkie: added queue_size argumet to the publishers
* multimaster_fkie: removed some python mistakes
* Contributors: Alexander Tiderko

0.3.14 (2014-10-24)
-------------------

0.3.13 (2014-07-29)
-------------------

0.3.12 (2014-07-08)
-------------------

0.3.11 (2014-06-04)
-------------------
* master_discovery_fkie: added some error catches to solve problems with removing of interfaces
* master_discovery_fkie: fixed a short timestamp represantation
* Contributors: Alexander Tiderko

0.3.10 (2014-03-31)
-------------------
* master_discovery_fkie: unsubscribe from parameter at the end
* master_discovery_fkie: remove invalid roslaunch uris from ROS Master
* multimaster_fkie: fixed problems detected by catkin_lint

0.3.9 (2013-12-12)
------------------
* master_discovery_fkie: added warning outputs on errors
* multimaster_fkie: moved .gitignore to top level

0.3.8 (2013-12-10)
------------------
* master_discovery_fkie: added output info about approx. netload
* multimaster_fkie: added a possibility to deaktivate the multicast heart bearts
* master_discovery_fkie: description in der package.xml changed

0.3.7 (2013-10-17)
------------------
* multimaster_fkie: fixed problems with resolving service types while sync
  while synchronization not all topics and services can be synchronized
  because of filter or errors. A detection for this case was added.
* node_manager_fkie: fix node matching
* master_discovery_fkie: do not publish not resolvable ROS MASTER URI to own ROS network

0.3.6 (2013-09-17)
------------------
* multimaster_fkie: added SyncServiceInfo message to detect changes on services
* master_discovery_fkie: fixed the origin master uri for services
* master_discovery_fkie: fixed the result of the synchronized nodes (handle the restart of the nodes without stop the running node)
* master_discovery_fkie: fixed the test of local changes -> not all changes are propageted to other hosts

0.3.5 (2013-09-06)
------------------

0.3.4 (2013-09-05)
------------------

0.3.3 (2013-09-04)
------------------
* master_discovery_fkie: fixed out, if the ROS_MASTER_URI refs to 'localhost'
* master_discovery_fkie: fixed the load interface
* multimaster_fkie: (*) added additional filtered interface to master_discovery rpc-server to get a filtered MasterInfo and reduce the load on network.
  (*) added the possibility to sync remote nodes using ~sync_remote_nodes parameter
