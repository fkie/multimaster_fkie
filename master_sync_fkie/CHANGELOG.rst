^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package master_sync_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
