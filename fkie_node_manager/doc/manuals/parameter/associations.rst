Associations
------------

Some ROS-Node relationships always require a common restart. If *roslaunch* is used to execute a launch file, this is done automatically.
In contrast to *roslaunch*, with the *Node Manager* the ROS-Nodes can be started or stopped individually.
To avoid errors in use because a dependent ROS-Node was not restarted, an **associations** parameter was introduced.
Add this parameter to your node and list all the nodes which should be started or stopped with this node, e.g.::

    <node name="YOUR_NODE_NAME" pkg="...
        <param name="nm/associations" value="NODE1,NODE2" />
    </node>


**Node names**:

Node names beginning with **/** remain unchanged. On relative name, the namespace of the node is appended in front of it.


**Start/Stop behaviour**:

Associated ROS-Nodes are started **before** the node itself and stopped **after** the node.


.. tip::
   All *Node Manager* specific parameters can be selected in the launch editor. For details see `launch editor`_.

.. _`launch editor`: editor.rst
