Logs of the nodes
-----------------

The ROS-Nodes started throuth the *Node Manager* are launched in it's own **screen**.
The output of this screen is stored to a file located in `~/.ros/log`.


There are different options to see the output of selected node:

    - Open screen terminal:
        This works only with running nodes.
        Press **F3** or |show_screen| button on the right site.
        In open terminal you can navigate using `screen key bindings`_.

        Be aware: ROS-Node blocks on output while you are in **copy mode**.

    - Internal Log-Viewer:
        **Double-Click** on the node opens the log file in a new dockable widget.
        See `logscreen`_ for all features.

    - Open screen log:
        Press |show_log|-button on the right site. It opens the log file in a new terminal.
        Since the log file is not cleared on the launch of the node you need to scroll to the end to see last output.

    - Copy path of the log file:
        Select the node, open **Node - Info** docking widget and click on |copy_path| icon.
        Use it to open the logfile in your preffered viewer.


.. _`screen key bindings`: https://www.gnu.org/software/screen/manual/screen.html#Default-Key-Bindings
.. |show_screen| image:: ../../images/crystal_clear_show_io.png
.. |show_log| image:: ../../images/crystal_clear_show_log.png
.. |copy_path| image:: ../../images/crystal_clear_copy_log_path_24.png
.. _`logscreen`: logscreen.rst
