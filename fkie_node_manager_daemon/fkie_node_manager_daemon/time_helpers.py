# ROS 2 Node Manager
# Graphical interface to manage the running and configured ROS 2 nodes on different hosts.
#
# Author: Alexander Tiderko
#
# Copyright 2020 Fraunhofer FKIE
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from rclpy.time import Time, CONVERSION_CONSTANT


def rostime2float(rcltime: Time):
    return float('.'.join(str(ele) for ele in rcltime.seconds_nanoseconds()))


def float2rostime(value: float):
    seconds = int(value)
    nanoseconds = (value - seconds) * CONVERSION_CONSTANT
    return Time(seconds=seconds, nanoseconds=nanoseconds)
