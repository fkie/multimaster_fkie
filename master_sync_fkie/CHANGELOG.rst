^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package master_sync_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
