^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fkie_master_discovery
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.7 (2021-01-23)
------------------

1.2.6 (2021-01-16)
------------------
* fixed "RuntimeError: dictionary changed size during iteration", issue `#150 <https://github.com/fkie/multimaster_fkie/issues/150>`_
* replaced Thread.isAlive() by .is_alive() according to issue `#149 <https://github.com/fkie/multimaster_fkie/issues/149>`_
* Contributors: Alexander Tiderko

1.2.5 (2020-11-28)
------------------

1.2.4 (2020-11-11)
------------------
* fkie_master_discovery: fix discovery in some cases with multicast dropped while running
* fkie_master_discovery: removed map of local addresses to localhost
  This caused problems if ROS_IP of ROS_HOSTNAME was used
* master_discovery: added parameter to hide nodes, topics and services for filtered requests
* Contributors: Alexander Tiderko

1.2.0 (2020-07-22)
------------------
* fkie_master_discovery: fixed threading problem
* fkie_master_discovery: fixed shutdown process
  set daemon_threads to True in ThreadingMixIn
* fkie_master_discovery: reduced timeout for connection requests
* fkie_master_discovery: added exception on wrong message size
* fkie_multimaster: fixed warning for cmake_minimum_required
* fkie_multimaster: fixed build/start in noetic
* fkie_master_discovery: do not publish statistics after shutdown
* fkie_master_discovery: fixed python3 compatibility on close
* master_discovery: added more output details to find problems on issue `#130 <https://github.com/fkie/multimaster_fkie/issues/130>`_
  more details for "master_discovery node appear not to running"
* Contributors: Alexander Tiderko

1.1.0 (2020-05-13)
------------------
* prepared conditions for python3  in package xml
* fkie_multimaster_msgs: changed timestamp in MasterState from float to time
* fkie_multimaster_msgs: changed timestamp in LinkState from float to time
* fkie_multimaster: added timestamp of last heartbeat to LinkState message
* fkie_multimaster_fkie: got get_local_address from rosgraph.network
* fkie_multimaster: a lot of merges for python 3 compatibility
* Contributors: Alexander Tiderko

0.8.12 (2019-04-30)
-------------------
* Merge pull request `#100 <https://github.com/fkie/multimaster_fkie/issues/100>`_ from stertingen/patch-1
  zeroconf.py: Detect IPv6 usage from environment
* zeroconf.py: Detect IPv6 usage from environment
  Set environment ROS_IPV6=on to enable the IPv6 RPC server.
* master_discovery_fkie: zeroconf: added fqdn-parameter, see issue `#99 <https://github.com/fkie/multimaster_fkie/issues/99>`_
* master_discovery_fkie: fixed some format warning
* master_discovery_fkie: zeroconf use for monitoruri the same hostname from masteruri
* Contributors: Alexander Tiderko, Hermann von Kleist

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

0.8.4 (2018-12-08)
------------------
* master_discovery_fkie: fix zeroconf to avoid request loop in master_sync
  see issue `#90 <https://github.com/fkie/multimaster_fkie/issues/90>`_
* Contributors: Alexander Tiderko

0.8.3 (2018-12-07)
------------------
* Fixed typo hearbeat[s] to heartbeat[s] pull request `#87 <https://github.com/fkie/multimaster_fkie/issues/87>`_ from Paulls20/master
* Contributors: Alexander Tiderko, Paul.Varghese

0.8.2 (2018-08-10)
------------------

0.8.1 (2018-08-03)
------------------

0.8.0 (2018-07-16)
------------------

0.7.8 (2018-03-24)
------------------
* Fix catkin_lint warnings
* Merge pull request `#69 <https://github.com/fkie/fkie_multimaster/issues/69>`_ from AlexisTM/fix_exit_zeroconf
  Solve zeroconf sys.exit( ..., ...) issue
* Contributors: Alexander Tiderko, Alexis Paques, Timo Röhling

0.7.7 (2017-10-27)
------------------

0.7.6 (2017-10-04)
------------------

0.7.5 (2017-07-17)
------------------
* fkie_master_discovery: reduced warning outputs in cases a node or service is not reachable
* Contributors: Alexander Tiderko

0.7.4 (2017-05-03)
------------------
* fkie_master_discovery: improved filter logging
* fixed read parameter with host filter
* Contributors: Alexander Tiderko

0.7.3 (2017-04-24)
------------------
* fixed warnings in API documentation
* Contributors: Alexander Tiderko

0.7.2 (2017-01-27)
------------------
* fkie_master_discovery: reverted the cut of domains in hostnames
* Contributors: Alexander Tiderko

0.7.1 (2017-01-26)
------------------
* fkie_master_discovery: fixed some problems on macOS
	- perform test for multicast interfaces only on Linux and FreeBSD
	- changed detection for local interface to support discovering on iOS
* fkie_master_discovery: removed domain suffix from hostname
* fkie_master_discovery: removed a not needed import
* fkie_master_discovery: digrammar fix in exception message
* Contributors: Alexander Tiderko, Jason Mercer

0.7.0 (2017-01-09)
------------------
* fkie_master_discovery: added detection for timejumps into the past
* fkie_master_discovery: fixed the shutdown process
    sometimes blocks the SimpleXMLRPCServer the shutdown process. Added a
    timer to kill the own process at the end.
* fkie_master_discovery: `#55 <https://github.com/fkie/fkie_multimaster/issues/55>`_ change the message handling routines
  Introduced a send and receive Queue. It was need to implement new
  features like hub/client structure.
  Added more debug output.
* fkie_master_discovery: splitted send_mcast into send_mcast and listen_mcast to get a hub functionality
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
* fix for `#46 <https://github.com/fkie/fkie_multimaster/issues/46>`_: bouncing offline/online
  reduced discovery heartbeats, especially if one of the masters is not reachable anymore.
* Contributors: Alexander Tiderko

0.5.7 (2016-09-07)
------------------

0.5.6 (2016-09-01)
------------------

0.5.5 (2016-08-30)
------------------
* fkie_master_discovery: fixed issue`#16 <https://github.com/fkie/fkie_multimaster/issues/16>`_
* fkie_multimaster: changed indent in source code to 4
* fkie_master_discovery: added network separation to zeroconf discovering
* fkie_master_discovery: changed the ROS service initialization
  The ROS service will be created after discovering process is started.
  This is especially for visualisation in node_manager.
* fkie_multimaster: removed unused imports
* fkie_master_discovery: fixed pep8 warnings
* fkie_master_discovery: replaced time.sleep by threading.Timer to handle connection problems while get remote master info
* master_discover_fkie: added warning on send errors
* fkie_master_discovery: removed '-' from master name generation for ROS master with not default port
* fkie_master_discovery: reduced/changed log output
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
* Use ROS_HOSTNAME before ROS_IP.
  To determine which interface to use follow ROS master convention
  of ROS_HOSTNAME being used before ROS_IP.
  This is as per:
  http://wiki.ros.org/ROS/EnvironmentVariables#ROS_IP.2BAC8-ROS_HOSTNAME
* Contributors: Mike O'Driscoll, Timo Röhling

0.5.0 (2016-03-17)
------------------
* master_discovery: fixed avg. network load calculation, added checks for some parameters
* fkie_multimaster: Set correct logging level to warning
* Contributors: Alexander Tiderko, Gary Servin

0.4.4 (2015-12-18)
------------------

0.4.3 (2015-11-30)
------------------
* fkie_master_discovery: fixed compatibility to older versions
* master_fiscovery_fkie: integrated pull request `#24 <https://github.com/fkie/fkie_multimaster/issues/24>`_
  Thanks for creating the PR to @garyservin and @mikeodr!
  The change lets you define an interface by `~interface`, `ROS_IP` envar
  or append the interface to multicast group like
  226.0.0.0@192.168.101.10. The master_discovery then binds to the
  specified interface and creates also an unicast interface for active
  requests on communication problems or if `~robot_hosts` are defined.
  Now you can also disable the multicast communication by setting
  `~send_mcast` to false. In this case the requests are send to hosts
  defined in `~robot_hosts`.
* fkie_master_discovery: fixed the 'local' assignment while updateInfo()
* fkie_master_discovery: adopt some changes from pull request `#24 <https://github.com/fkie/fkie_multimaster/issues/24>`_
  Thanks to @garyservin for pull request `#24 <https://github.com/fkie/fkie_multimaster/issues/24>`_:
  * Don't exit if we're on localhost, just log a warning
  * Added support for different logging levels in master_monitor:
  currently all logs are marked as warnings, where some should be marked
  as errors.
* fkie_multimaster: reduced logs and warnings on stop nodes while closing node_manager
* fkie_multimaster: reduced logging of exceptions
* fkie_master_discovery: spaces and typos removed
* fkie_master_discovery: fixed link quality calculation
* Contributors: Alexander Tiderko

0.4.2 (2015-10-19)
------------------
* fkie_master_discovery: fixed the updates of remote nodes registered on local master
* fkie_multimaster: added a possibility to set time on remote host
* fkie_node_manager: added a warning if the time difference to remote host is greater than a defined value (default 3 sec)
* fkie_master_discovery: added @part to define interface with mcast group
* fkie_master_discovery: add posibility to specify the interface to use
* master_discover_fkie: check for local ip addresses to avoid wrong warning messages
* Contributors: Alexander Tiderko

0.4.1 (2015-04-28)
------------------
* Deprecate is_ignored_topic. Move new parameters to the end of the parameter list
* Make configuration more granular
  allows filtering of specific subscribers or publishers
* fkie_multimaster: fixed double log output
* fkie_multimaster: added network problem detection on remote hosts
* fkie_multimaster: fixed error in launch files included in this package
* Contributors: Alexander Tiderko, Julian Cerruti

0.4.0 (2015-02-20)
------------------
* fkie_master_discovery: discovery changed
  * reduced the amount of heartbeat messages for discovery
  * added fallback for environments with multicast problems
* fkie_master_discovery: added log_level parameter to all nodes
* fkie_master_discovery: changed discovery after the host was set to offline
* fkie_master_discovery: fixed a problem if more then one master discovery is running on the same host
* fkie_master_discovery: removed some python mistakes
* fkie_master_discovery: removed some debug output
* fkie_master_discovery: fixed change to offline state after a refresh service was called and host is not reachable
* fkie_master_discovery: fix set to offline state
* fkie_master_discovery: fixed link quality detection.
  The requests for each master are now stored, to detect the right count
  of messages that we have to receive.
* Contributors: Alexander, Alexander Tiderko, Robot User

0.3.18 (2015-02-18)
-------------------

0.3.17 (2015-01-22)
-------------------
* fkie_master_discovery: fixed discovery support for ipv6
* Contributors: Alexander Tiderko

0.3.16 (2014-12-08)
-------------------

0.3.15 (2014-12-01)
-------------------
* fkie_multimaster: added queue_size argumet to the publishers
* fkie_multimaster: removed some python mistakes
* Contributors: Alexander Tiderko

0.3.14 (2014-10-24)
-------------------

0.3.13 (2014-07-29)
-------------------

0.3.12 (2014-07-08)
-------------------

0.3.11 (2014-06-04)
-------------------
* fkie_master_discovery: added some error catches to solve problems with removing of interfaces
* fkie_master_discovery: fixed a short timestamp represantation
* Contributors: Alexander Tiderko

0.3.10 (2014-03-31)
-------------------
* fkie_master_discovery: unsubscribe from parameter at the end
* fkie_master_discovery: remove invalid roslaunch uris from ROS Master
* fkie_multimaster: fixed problems detected by catkin_lint

0.3.9 (2013-12-12)
------------------
* fkie_master_discovery: added warning outputs on errors
* fkie_multimaster: moved .gitignore to top level

0.3.8 (2013-12-10)
------------------
* fkie_master_discovery: added output info about approx. netload
* fkie_multimaster: added a possibility to deaktivate the multicast heart bearts
* fkie_master_discovery: description in der package.xml changed

0.3.7 (2013-10-17)
------------------
* fkie_multimaster: fixed problems with resolving service types while sync
  while synchronization not all topics and services can be synchronized
  because of filter or errors. A detection for this case was added.
* fkie_node_manager: fix node matching
* fkie_master_discovery: do not publish not resolvable ROS MASTER URI to own ROS network

0.3.6 (2013-09-17)
------------------
* fkie_multimaster: added SyncServiceInfo message to detect changes on services
* fkie_master_discovery: fixed the origin master uri for services
* fkie_master_discovery: fixed the result of the synchronized nodes (handle the restart of the nodes without stop the running node)
* fkie_master_discovery: fixed the test of local changes -> not all changes are propageted to other hosts

0.3.5 (2013-09-06)
------------------

0.3.4 (2013-09-05)
------------------

0.3.3 (2013-09-04)
------------------
* fkie_master_discovery: fixed out, if the ROS_MASTER_URI refs to 'localhost'
* fkie_master_discovery: fixed the load interface
* fkie_multimaster: (*) added additional filtered interface to master_discovery rpc-server to get a filtered MasterInfo and reduce the load on network.
  (*) added the possibility to sync remote nodes using ~sync_remote_nodes parameter
