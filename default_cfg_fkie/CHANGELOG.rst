^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package default_cfg_fkie
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
