Node Manager Daemon is an instance which allows the Node Manager a remote access to configuration files . Through the daemon the launch file can be edited, loaded and the containing nodes executed by Node Manager GUI.

The daemon instance is usually launched be Node Manager through SSH connection. After that the Node Manager communicates with daemon using [gRPC](https://grpc.io/).

Beside offering remote configuration access to Node Manager the daemon supports many other features, e.g. system monitoring, forwarding diagnostic messages or auto start/load of launchfiles.

The configuration is stored at *$HOME/.config/ros.fkie/node_manager_daemon.yaml* and can be changed through Node Manager for each host.
