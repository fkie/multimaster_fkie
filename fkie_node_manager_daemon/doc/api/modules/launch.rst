`launch_config`
===============

Launch configuration is the class to load the ROS-Launch while starting nodes.

.. automodule:: fkie_node_manager_daemon.launch_config
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launch_description`
====================

Class definition for reporting the description of nodes to Node Manager.

.. automodule:: fkie_node_manager_daemon.launch_description
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launch_servicer`
=================

Handles the launch file functionality of GRPC Server defined in `fkie_multimaster_msgs/protos/launch.proto`.

.. automodule:: fkie_node_manager_daemon.launch_servicer
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launch_stub`
=============

Helper class for client to call the GRPC-methods of `LaunchServicer`.

.. automodule:: fkie_node_manager_daemon.launch_stub
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launcher`
==========

Methods to extract all important values and launch a node in a screen.

.. automodule:: fkie_node_manager_daemon.launcher
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`startcfg`
==========

Helper class with all parameter to launch a node. All parameter a public and are set by caller instance.

.. automodule:: fkie_node_manager_daemon.startcfg
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`supervised_popen`
==================

Helper class waits for process, which starts nodes in a screen to avoid 'defunct' processes.

.. automodule:: fkie_node_manager_daemon.supervised_popen
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

