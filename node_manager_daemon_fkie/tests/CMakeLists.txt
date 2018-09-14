### Unit tests
#
#   Only run when CATKIN_ENABLE_TESTING is true.

##  Python

# Unit tests not needing a running ROS core.
catkin_add_nosetests(test_common.py)

# Unit tests using nose, but needing a running ROS core.
find_package(rostest)
if (rostest_FOUND)
#  add_rostest(test_tf_plugin_location.test)
#  add_rostest(test_tf_plugin_sniper.test)
endif()