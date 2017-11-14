^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package master_discovery_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.7 (2017-10-27)
------------------

0.7.6 (2017-10-04)
------------------

0.7.5 (2017-07-17)
------------------
* master_discovery_fkie: reduced warning outputs in cases a node or service is not reachable
* Contributors: Alexander Tiderko

0.7.4 (2017-05-03)
------------------
* master_discovery_fkie: improved filter logging
* fixed read parameter with host filter
* Contributors: Alexander Tiderko

0.7.3 (2017-04-24)
------------------
* fixed warnings in API documentation
* Contributors: Alexander Tiderko

0.7.2 (2017-01-27)
------------------
* master_discovery_fkie: reverted the cut of domains in hostnames
* Contributors: Alexander Tiderko

0.7.1 (2017-01-26)
------------------
* master_discovery_fkie: fixed some problems on macOS
	- perform test for multicast interfaces only on Linux and FreeBSD
	- changed detection for local interface to support discovering on iOS
* master_discovery_fkie: removed domain suffix from hostname
* master_discovery_fkie: removed a not needed import
* master_discovery_fkie: digrammar fix in exception message
* Contributors: Alexander Tiderko, Jason Mercer

0.7.0 (2017-01-09)
------------------
* master_discovery_fkie: added detection for timejumps into the past
* master_discovery_fkie: fixed the shutdown process
    sometimes blocks the SimpleXMLRPCServer the shutdown process. Added a
    timer to kill the own process at the end.
* master_discovery_fkie: `#55 <https://github.com/fkie/multimaster_fkie/issues/55>`_ change the message handling routines
  Introduced a send and receive Queue. It was need to implement new
  features like hub/client structure.
  Added more debug output.
* master_discovery_fkie: splitted send_mcast into send_mcast and listen_mcast to get a hub functionality
* Contributors: Alexander Tiderko, deng02

0.6.2 (2016-11-12)
------------------
* Drop roslib.load_manifest, unneeded with catkin
* Contributors: Alexander Tiderko, Mike Purvis

0.6.1 (2016-10-18)
------------------
* fix for issue #50: do not sent and reply requests while own state is not available
* Contributors: Alexander Tiderko

0.6.0 (2016-10-12)
------------------

0.5.8 (2016-09-10)
------------------
* fix for `#46 <https://github.com/fkie/multimaster_fkie/issues/46>`_: bouncing offline/online
  reduced discovery heartbeats, especially if one of the masters is not reachable anymore.
* Contributors: Alexander Tiderko

0.5.7 (2016-09-07)
------------------

0.5.6 (2016-09-01)
------------------

0.5.5 (2016-08-30)
------------------
* master_discovery_fkie: fixed issue`#16 <https://github.com/fkie/multimaster_fkie/issues/16>`_
* multimaster_fkie: changed indent in source code to 4
* master_discovery_fkie: added network separation to zeroconf discovering
* master_discovery_fkie: changed the ROS service initialization
  The ROS service will be created after discovering process is started.
  This is especially for visualisation in node_manager.
* multimaster_fkie: removed unused imports
* master_discovery_fkie: fixed pep8 warnings
* master_discovery_fkie: replaced time.sleep by threading.Timer to handle connection problems while get remote master info
* master_discover_fkie: added warning on send errors
* master_discovery_fkie: removed '-' from master name generation for ROS master with not default port
* master_discovery_fkie: reduced/changed log output
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
* Use ROS_HOSTNAME before ROS_IP.
  To determine which interface to use follow ROS master convention
  of ROS_HOSTNAME being used before ROS_IP.
  This is as per:
  http://wiki.ros.org/ROS/EnvironmentVariables#ROS_IP.2BAC8-ROS_HOSTNAME
* Contributors: Mike O'Driscoll, Timo Röhling

0.5.0 (2016-03-17)
------------------
* master_discovery: fixed avg. network load calculation, added checks for some parameters
* multimaster_fkie: Set correct logging level to warning
* Contributors: Alexander Tiderko, Gary Servin

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
