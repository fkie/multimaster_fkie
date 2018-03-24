^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package master_sync_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* master_sync_fkie: changed default filter for sync nodes, see issue `#63 <https://github.com/fkie/multimaster_fkie/issues/63>`_
* Contributors: Alexander Tiderko

0.7.4 (2017-05-03)
------------------
* master_sync_fkie: fixed sync_hosts parameter
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
* master_sync_fkie: updated launch file
* master_sync_fkie: added a 'resync_on_reconnect_timeout' parameter that controls how long the offline-online period is before the resync. see enhancement `#48 <https://github.com/fkie/multimaster_fkie/issues/48>`_
* Contributors: Alexander Tiderko

0.5.8 (2016-09-10)
------------------

0.5.7 (2016-09-07)
------------------

0.5.6 (2016-09-01)
------------------

0.5.5 (2016-08-30)
------------------
* master_sync_fkie: added resync after the host was offline
* master_sync_fkie: fixed pep8 warnings
* Contributors: Alexander Tiderko

0.5.4 (2016-04-21)
------------------
* multimaster_fkie: added '/do_not_sync' parameter
  this allows to hide some topics/services, topic types, from
  synchronisation. It can be defined as string or as list.
* master_sync_fkie: fixed unnecessary update requests
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
* master_discovery_fkie: adopt some changes from pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_
  Thanks to @garyservin for pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_:
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
* multimaster_fkie: fixed double log output
* multimaster_fkie: fixed error in launch files included in this package
* Contributors: Alexander Tiderko, Julian Cerruti

0.4.0 (2015-02-20)
------------------
* multimaster_fkie: added log_level parameter to all nodes
* master_sync: fix the long wait time on first sync
* master_sync_fkie: fix annonce publisher about the AnyMsg subscribers
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
* multimaster_fkie: added queue_size argumet to the publishers
* multimaster_fkie: removed some python mistakes
* Contributors: Alexander Tiderko

0.3.14 (2014-10-24)
-------------------
* master_sync_fkie: reduced update notifications after registration of a subscriber

0.3.13 (2014-07-29)
-------------------

0.3.12 (2014-07-08)
-------------------

0.3.11 (2014-06-04)
-------------------
* master_sync_fkie: fixed a block while connection problems

0.3.10 (2014-03-31)
-------------------
* master_sync_fkie: fixed a bug which sometimes does not synchronized some topics
* multimaster_fkie: fixed problems detected by catkin_lint

0.3.9 (2013-12-12)
------------------
* multimaster_fkie: moved .gitignore to top level

0.3.8 (2013-12-10)
------------------
* master_sync_fkie: added sync for subscriber with AnyMsg, e.g relay (topic_tools), if local a publisher with known type is available
* multimaster_fkie: catkin_lint inspired fixes, thanks @roehling

0.3.7 (2013-10-17)
------------------
* multimaster_fkie: fixed problems with resolving service types while sync
  while synchronization not all topics and services can be synchronized
  because of filter or errors. A detection for this case was added.

0.3.6 (2013-09-17)
------------------
* multimaster_fkie: added SyncServiceInfo message to detect changes on services
* master_sync_fkie: kill the own ros node on error while load interface to inform the user in node_manager about errors

0.3.5 (2013-09-06)
------------------
* master_sync_fkie: fixed a brocken connection after desync

0.3.4 (2013-09-05)
------------------

0.3.3 (2013-09-04)
------------------
* node_manager_fkie: fixed a problem while launching a default cfg nodes
* multimaster_fkie: (*) added additional filtered interface to master_discovery rpc-server to get a filtered MasterInfo and reduce the load on network.
  (*) added the possibility to sync remote nodes using ~sync_remote_nodes parameter
* master_sync_fkie: added support to ignore nodes/topic/services of selected hosts
* master_sync_fkie: fixed ignore hosts, some topics sync ignores
