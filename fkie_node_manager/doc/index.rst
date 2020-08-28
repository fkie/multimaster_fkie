
Node Manager
============

A GUI to manage the configuration on local and remote ROS masters. For more details see manual_, roswiki_, API_ or use the **(?)** button in the title bar.

For configuration click on |settings| icon in bottom middle of the Node Manager.

.. _manual: http://fkie.github.io/multimaster_fkie/
.. _roswiki: http://www.ros.org/wiki/node_manager_fkie
.. _API: html/index.html
.. |settings| image:: images/crystal_clear_settings_24.png


Manual
------
* Logging
    * `Logs of the nodes`_
    * Logscreen_
* Parameter
    * `Custom parameter in Launch Editor`_
    * Associations_


References
----------

shortcuts_

changelog_


.. _`Logs of the nodes`: manuals/logging/logs.rst
.. _Logscreen: manuals/logging/logscreen.rst
.. _`Custom parameter in Launch Editor`: manuals/parameter/editor.rst
.. _Associations: manuals/parameter/associations.rst
.. _shortcuts: manuals/shortcuts.rst
.. _changelog: ../CHANGELOG.rst

````


**Troubleshooting**

- Error while launch a node on remote host: ``bash: rosrun: command not found``

    To run a node on remote host, an SSH connection will be established without setting any enviroment variables.

    Add ``source /opt/ros/xxx/setup.bash`` to **.bashrc** before ``[ -z "$PS1" ] && return``

- The Node Manager crashes on load a launch file with error: *QSpiAccessible::accessibleEvent not handled: "8008"*

    This "bug" seems to be resolved by removing the **qt-at-spi** package.

- You don't see the correct output of your nodes. Try to change your default terminal:

    ``sudo update-alternatives --config x-terminal-emulator``

- You get an exception on access remote host: *Exception: ssh connection to REMOTE_HOST failed: not a valid RSA private key file*

    Generate an SSH key file with e.g. ``ssh-keygen -p -m PEM -f ~/.ssh/id_rsa``