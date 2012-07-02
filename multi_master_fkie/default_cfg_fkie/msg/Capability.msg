# one of the ID_* defines
# if the id of the message is sensor, then the ros_name contains full ROS node
# name, empty string otherwise
# type of the sensor, defined in launch file using sensor_type parameter tag.
# name of the sensor, defined in launch file using sensor_name parameter tag.
# description of the sensor, defined in launch file using sensor_description parameter tag.
#string ID_ROBOT=robot
#string ID_SENSOR=sensor

string namespace
string name
string type
string[] images
string description
string[] nodes 