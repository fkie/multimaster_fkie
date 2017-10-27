^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package default_cfg_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.7 (2017-10-27)
------------------

0.7.6 (2017-10-04)
------------------

0.7.5 (2017-07-17)
------------------
* default_cfg_fkie: store the arguments of default_cfg to parameter server
* Contributors: Alexander Tiderko

0.7.4 (2017-05-03)
------------------

0.7.3 (2017-04-24)
------------------
* default_cfg_fkie: fixed problem with "pass_all_args" attribute
* fixed warnings in API documentation
* Contributors: Alexander Tiderko

0.7.2 (2017-01-27)
------------------

0.7.1 (2017-01-26)
------------------

0.7.0 (2017-01-09)
------------------
* default_cfg_fkie: fixed start nodes with same name and different namespaces
* default_cfg_fkie: fix the namespace for rqt-cpp-plugins
* Contributors: Alexander Tiderko

0.6.2 (2016-11-12)
------------------
* Drop roslib.load_manifest, unneeded with catkin
* Contributors: Alexander Tiderko, Mike Purvis

0.6.1 (2016-10-18)
------------------

0.6.0 (2016-10-12)
------------------

0.5.8 (2016-09-10)
------------------

0.5.7 (2016-09-07)
------------------

0.5.6 (2016-09-01)
------------------

0.5.5 (2016-08-30)
------------------
* multimaster_fkie: changed indent in source code to 4
* default_cfg_fkie: removed pep8 warnings
* multimaster_fkie: removed unused imports
* Contributors: Alexander Tiderko

0.5.4 (2016-04-21)
------------------

0.5.3 (2016-04-01)
------------------

0.5.2 (2016-03-31)
------------------

0.5.1 (2016-03-23)
------------------

0.5.0 (2016-03-17)
------------------
* default_cfg_fkie: added 'load_params_at_start' parameter.
  On start of default_cfg_fkie all parameters are loaded into ROS
  parameter server. If this parameter is set to `False` the parameter are
  loaded on first run of an included node.
* mulitmaster_fkie: fixed the missing leading SEP by groups
* Contributors: Alexander Tiderko

0.4.4 (2015-12-18)
------------------

0.4.3 (2015-11-30)
------------------

0.4.2 (2015-10-19)
------------------

0.4.1 (2015-04-28)
------------------
* multimaster_fkie: fixed double log output
* Contributors: Alexander Tiderko

0.4.0 (2015-02-20)
------------------
* multimaster_fkie: added log_level parameter to all nodes
* Contributors: Alexander Tiderko

0.3.18 (2015-02-18)
-------------------

0.3.17 (2015-01-22)
-------------------

0.3.16 (2014-12-08)
-------------------

0.3.15 (2014-12-01)
-------------------
* multimaster_fkie: removed some python mistakes
* Contributors: Alexander Tiderko

0.3.14 (2014-10-24)
-------------------
* node_manager_fkie: added support of $(find ...) statement to add images in decription of capabilities

0.3.13 (2014-07-29)
-------------------

0.3.12 (2014-07-08)
-------------------

0.3.11 (2014-06-04)
-------------------
* default_cfg_fkie: added 'default_cfg/autostart/exclude' parameter to exclude nodes from autostart
* default_cfg_fkie: flush stdout before SIGKILL
  Otherwise, the error message may not reach the console output in time.
* default_cfg_fkie: added a console output for count of pending autostart nodes
* default_cfg_fkie: set autostart to False after all node are started
* default_cfg_fkie: added a reload service, to reload the configuration
* default_cfg_fkie: added for each node respawn parameters
* default_cfg_fkie: added a possibility for delayd start or after a published topic is available
* default_cfg_fkie: loads now without the private namespace of the default_cfg node
* default_cfg_fkie: removed BASH_ENV from environment while start with respawn script
* default_cfg_fkie: added an autostart option
* Contributors: Alexander Tiderko, Timo Röhling

0.3.10 (2014-03-31)
-------------------
* multimaster_fkie: fixed problems detected by catkin_lint

0.3.9 (2013-12-12)
------------------
* multimaster_fkie: moved .gitignore to top level

0.3.8 (2013-12-10)
------------------
* default_cfg_fkie: fixed forward error to service caller
* default_cfg_fkie: ignore empty 'capability_group' values
* multimaster_fkie: catkin_lint inspired fixes, thanks @roehling

0.3.7 (2013-10-17)
------------------
* default_cfg_fkie: fixed parameter groups

0.3.6 (2013-09-17)
------------------
* default_cfg_fkie: changed the intepretation of the group description

0.3.5 (2013-09-06)
------------------

0.3.4 (2013-09-05)
------------------
* default_cfg_fkie: capability_group parameter can now be defined in a namespace
