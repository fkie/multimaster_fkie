python -m grpc_tools.protoc -Iprotos --python_out=./src/node_manager_daemon_fkie/generated/. --grpc_python_out=./src/node_manager_daemon_fkie/generated/. protos/file.proto 
python -m grpc_tools.protoc -Iprotos --python_out=./src/node_manager_daemon_fkie/generated/. --grpc_python_out=./src/node_manager_daemon_fkie/generated/. protos/launch.proto
python -m grpc_tools.protoc -Iprotos --python_out=./src/node_manager_daemon_fkie/generated/. --grpc_python_out=./src/node_manager_daemon_fkie/generated/. protos/screen.proto 
