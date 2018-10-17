`launch_config`
===============

Launch configuration is the class to load the ROS-Launch while starting nodes.

.. automodule:: node_manager_daemon_fkie.launch_config
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launch_description`
====================

Class definition for reporting the description of nodes to Node Manager.

.. automodule:: node_manager_daemon_fkie.launch_description
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launch_servicer`
=================

Handles the launch file functionality of GRPC Server defined in `protos/launch.proto`.

.. automodule:: node_manager_daemon_fkie.launch_servicer
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launch_stub`
=============

Helper class for client to call the GRPC-methods of `LaunchServicer`.

.. automodule:: node_manager_daemon_fkie.launch_stub
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`launcher`
==========

Methods to extract all important values and launch a node in a screen.

.. automodule:: node_manager_daemon_fkie.launcher
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`startcfg`
==========

Helper class with all parameter to launch a node. All parameter a public and are set by caller instance.

.. automodule:: node_manager_daemon_fkie.startcfg
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

`supervised_popen`
==================

Helper class waits for process, which starts nodes in a screen to avoid 'defunct' processes.

.. automodule:: node_manager_daemon_fkie.supervised_popen
   :members:
   :undoc-members:
   :show-inheritance:
   :special-members: __init__

