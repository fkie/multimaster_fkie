# one of the ID_* defines
string id
# if the id of the message is sensor, then the ros_name contains full ROS node
# name, empty string otherwise
string ros_name
# type of the sensor, defined in launch file using sensor_type parameter tag.
string type
# name of the sensor, defined in launch file using sensor_name parameter tag.
string name
# description of the sensor, defined in launch file using sensor_description parameter tag.
string description

string ID_ROBOT=robot
string ID_SENSOR=sensor