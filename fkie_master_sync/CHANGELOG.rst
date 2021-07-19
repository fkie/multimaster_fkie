^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_master_sync
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.7 (2021-01-23)
------------------

1.2.6 (2021-01-16)
------------------
* replaced Thread.isAlive() by .is_alive() according to issue `#149 <https://github.com/fkie/multimaster_fkie/issues/149>`_
* Contributors: Alexander Tiderko

1.2.5 (2021-01-16)
------------------

1.2.4 (2020-11-11)
------------------
* fkie_master_sync: fix problems after stop master_sync on remote hosts
* updated diagnostic message for warnings in master_sync
* Contributors: Alexander Tiderko

1.2.0 (2020-07-22)
------------------
* fkie_multimaster: fixed warning for cmake_minimum_required
* fkie_multimaster: fixed build/start in noetic
* fkie_master_sync: send OK diagnostic message on start
* fkie_master_sync: changed handling of the issue `#128 <https://github.com/fkie/multimaster_fkie/issues/128>`_
  Changed update of the publisher after register a remote subscription
* Contributors: Alexander Tiderko

1.1.0 (2020-05-13)
------------------
* prepared conditions for python3  in package xml
* fkie_multimaster_msgs: changed timestamp in MasterState from float to time
* fkie_multimaster: a lot of merges for python 3 compatibility
* Contributors: Alexander Tiderko

0.8.12 (2019-04-30)
-------------------

0.8.11 (2019-02-27)
-------------------

0.8.10 (2019-02-26)
-------------------

0.8.9 (2018-12-21)
------------------

0.8.8 (2018-12-19)
------------------

0.8.7 (2018-12-18)
------------------

0.8.5 (2018-12-11)
------------------
* master_sync_fkie: added a simple node to sync parameter
  Original code from
  https://github.com/jhu-lcsr-forks/multimaster_fkie/tree/param-sync
  adapted to change only local ROS Parameter Server
* Contributors: Alexander Tiderko

0.8.4 (2018-12-08)
------------------

0.8.3 (2018-12-07)
------------------
* install launch dir pull request `#81 <https://github.com/fkie/multimaster_fkie/issues/81>`_ from ahoarau/patch-1
* Contributors: Alexander Tiderko, Antoine Hoarau

0.8.2 (2018-08-10)
------------------

0.8.1 (2018-08-03)
------------------

0.8.0 (2018-07-16)
------------------

0.7.8 (2018-03-24)
------------------
* Fix catkin_lint warnings
* Contributors: Timo Röhling

0.7.7 (2017-10-27)
------------------

0.7.6 (2017-10-04)
------------------

0.7.5 (2017-07-17)
------------------
* fkie_master_sync: changed default filter for sync nodes, see issue `#63 <https://github.com/fkie/fkie_multimaster/issues/63>`_
* Contributors: Alexander Tiderko

0.7.4 (2017-05-03)
------------------
* fkie_master_sync: fixed sync_hosts parameter
* added description how to filter for specific hosts
* Contributors: Alexander Tiderko

0.7.3 (2017-04-24)
------------------
* fixed warnings in API documentation
* Contributors: Alexander Tiderko

0.7.2 (2017-01-27)
------------------

0.7.1 (2017-01-26)
------------------

0.7.0 (2017-01-09)
------------------

0.6.2 (2016-11-12)
------------------
* Increased logging in master sync.
  Added more logging around synchronization to help with
  tracking changes in the local ROS master due to multimaster.
* Drop roslib.load_manifest, unneeded with catkin
* Contributors: Alexander Tiderko, Denise Eng, Mike Purvis

0.6.1 (2016-10-18)
------------------

0.6.0 (2016-10-12)
------------------
* fkie_master_sync: updated launch file
* fkie_master_sync: added a 'resync_on_reconnect_timeout' parameter that controls how long the offline-online period is before the resync. see enhancement `#48 <https://github.com/fkie/fkie_multimaster/issues/48>`_
* Contributors: Alexander Tiderko

0.5.8 (2016-09-10)
------------------

0.5.7 (2016-09-07)
------------------

0.5.6 (2016-09-01)
------------------

0.5.5 (2016-08-30)
------------------
* fkie_master_sync: added resync after the host was offline
* fkie_master_sync: fixed pep8 warnings
* Contributors: Alexander Tiderko

0.5.4 (2016-04-21)
------------------
* fkie_multimaster: added '/do_not_sync' parameter
  this allows to hide some topics/services, topic types, from
  synchronisation. It can be defined as string or as list.
* fkie_master_sync: fixed unnecessary update requests
  wrong timestamps leads to updates
* Contributors: Alexander Tiderko

0.5.3 (2016-04-01)
------------------

0.5.2 (2016-03-31)
------------------

0.5.1 (2016-03-23)
------------------

0.5.0 (2016-03-17)
------------------

0.4.4 (2015-12-18)
------------------

0.4.3 (2015-11-30)
------------------
* fkie_master_discovery: adopt some changes from pull request `#24 <https://github.com/fkie/fkie_multimaster/issues/24>`_
  Thanks to @garyservin for pull request `#24 <https://github.com/fkie/fkie_multimaster/issues/24>`_:
  * Added support for different logging levels in master_monitor:
  currently all logs are marked as warnings, where some should be marked
  as errors.
* Contributors: Alexander Tiderko

0.4.2 (2015-10-19)
------------------

0.4.1 (2015-04-28)
------------------
* Deprecate is_ignored_topic. Move new parameters to the end of the parameter list
* Make configuration more granular
  allows filtering of specific subscribers or publishers
* fkie_multimaster: fixed double log output
* fkie_multimaster: fixed error in launch files included in this package
* Contributors: Alexander Tiderko, Julian Cerruti

0.4.0 (2015-02-20)
------------------
* fkie_multimaster: added log_level parameter to all nodes
* master_sync: fix the long wait time on first sync
* fkie_master_sync: fix annonce publisher about the AnyMsg subscribers
* Contributors: Alexander Tiderko

0.3.18 (2015-02-18)
-------------------
* master_sync: subscribers with None type are now subscribed as AnyType message
* Contributors: Alexander Tiderko

0.3.17 (2015-01-22)
-------------------

0.3.16 (2014-12-08)
-------------------

0.3.15 (2014-12-01)
-------------------
* fkie_multimaster: added queue_size argumet to the publishers
* fkie_multimaster: removed some python mistakes
* Contributors: Alexander Tiderko

0.3.14 (2014-10-24)
-------------------
* fkie_master_sync: reduced update notifications after registration of a subscriber

0.3.13 (2014-07-29)
-------------------

0.3.12 (2014-07-08)
-------------------

0.3.11 (2014-06-04)
-------------------
* fkie_master_sync: fixed a block while connection problems

0.3.10 (2014-03-31)
-------------------
* fkie_master_sync: fixed a bug which sometimes does not synchronized some topics
* fkie_multimaster: fixed problems detected by catkin_lint

0.3.9 (2013-12-12)
------------------
* fkie_multimaster: moved .gitignore to top level

0.3.8 (2013-12-10)
------------------
* fkie_master_sync: added sync for subscriber with AnyMsg, e.g relay (topic_tools), if local a publisher with known type is available
* fkie_multimaster: catkin_lint inspired fixes, thanks @roehling

0.3.7 (2013-10-17)
------------------
* fkie_multimaster: fixed problems with resolving service types while sync
  while synchronization not all topics and services can be synchronized
  because of filter or errors. A detection for this case was added.

0.3.6 (2013-09-17)
------------------
* fkie_multimaster: added SyncServiceInfo message to detect changes on services
* fkie_master_sync: kill the own ros node on error while load interface to inform the user in node_manager about errors

0.3.5 (2013-09-06)
------------------
* fkie_master_sync: fixed a brocken connection after desync

0.3.4 (2013-09-05)
------------------

0.3.3 (2013-09-04)
------------------
* fkie_node_manager: fixed a problem while launching a default cfg nodes
* fkie_multimaster: (*) added additional filtered interface to master_discovery rpc-server to get a filtered MasterInfo and reduce the load on network.
  (*) added the possibility to sync remote nodes using ~sync_remote_nodes parameter
* fkie_master_sync: added support to ignore nodes/topic/services of selected hosts
* fkie_master_sync: fixed ignore hosts, some topics sync ignores
