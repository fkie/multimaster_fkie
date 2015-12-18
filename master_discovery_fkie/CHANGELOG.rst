^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package master_discovery_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.4 (2015-12-18)
------------------

0.4.3 (2015-11-30)
------------------
* master_discovery_fkie: fixed compatibility to older versions
* master_fiscovery_fkie: integrated pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_
  Thanks for creating the PR to @garyservin and @mikeodr!
  The change lets you define an interface by `~interface`, `ROS_IP` envar
  or append the interface to multicast group like
  226.0.0.0@192.168.101.10. The master_discovery then binds to the
  specified interface and creates also an unicast interface for active
  requests on communication problems or if `~robot_hosts` are defined.
  Now you can also disable the multicast communication by setting
  `~send_mcast` to false. In this case the requests are send to hosts
  defined in `~robot_hosts`.
* master_discovery_fkie: fixed the 'local' assignment while updateInfo()
* master_discovery_fkie: adopt some changes from pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_
  Thanks to @garyservin for pull request `#24 <https://github.com/fkie/multimaster_fkie/issues/24>`_:
  * Don't exit if we're on localhost, just log a warning
  * Added support for different logging levels in master_monitor:
  currently all logs are marked as warnings, where some should be marked
  as errors.
* multimaster_fkie: reduced logs and warnings on stop nodes while closing node_manager
* multimaster_fkie: reduced logging of exceptions
* master_discovery_fkie: spaces and typos removed
* master_discovery_fkie: fixed link quality calculation
* Contributors: Alexander Tiderko

0.4.2 (2015-10-19)
------------------
* master_discovery_fkie: fixed the updates of remote nodes registered on local master
* multimaster_fkie: added a possibility to set time on remote host
* node_manager_fkie: added a warning if the time difference to remote host is greater than a defined value (default 3 sec)
* master_discovery_fkie: added @part to define interface with mcast group
* master_discovery_fkie: add posibility to specify the interface to use
* master_discover_fkie: check for local ip addresses to avoid wrong warning messages
* Contributors: Alexander Tiderko

0.4.1 (2015-04-28)
------------------
* Deprecate is_ignored_topic. Move new parameters to the end of the parameter list
* Make configuration more granular
  allows filtering of specific subscribers or publishers
* multimaster_fkie: fixed double log output
* multimaster_fkie: added network problem detection on remote hosts
* multimaster_fkie: fixed error in launch files included in this package
* Contributors: Alexander Tiderko, Julian Cerruti

0.4.0 (2015-02-20)
------------------
* master_discovery_fkie: discovery changed
  * reduced the amount of heartbeat messages for discovery
  * added fallback for environments with multicast problems
* master_discovery_fkie: added log_level parameter to all nodes
* master_discovery_fkie: changed discovery after the host was set to offline
* master_discovery_fkie: fixed a problem if more then one master discovery is running on the same host
* master_discovery_fkie: removed some python mistakes
* master_discovery_fkie: removed some debug output
* master_discovery_fkie: fixed change to offline state after a refresh service was called and host is not reachable
* master_discovery_fkie: fix set to offline state
* master_discovery_fkie: fixed link quality detection.
  The requests for each master are now stored, to detect the right count
  of messages that we have to receive.
* Contributors: Alexander, Alexander Tiderko, Robot User

0.3.18 (2015-02-18)
-------------------

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
