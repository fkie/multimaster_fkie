
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

from typing import Text
from typing import Union

from datetime import datetime


def sizeof_fmt(num: Union[float, int], suffix: str = 'B') -> Text:
    for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
        if abs(num) < 1024.0:
            return '%.0f%s%s' % (num, unit, suffix)
        num /= 1024.0
    return '%.0f%s%s' % (num, 'YiB', suffix)


def timestamp_fmt(stamp: float, with_date: bool = True, with_nanosecs: bool = True, tz=None) -> Text:
    ts = stamp
    if hasattr(stamp, 'secs'):
        ts = stamp.secs + stamp.secs / 1000000000.
    str_format = '%H:%M:%S'
    if with_nanosecs:
        str_format += '.%f'
    if with_date:
        str_format += ' (%d.%m.%Y)'
    return datetime.fromtimestamp(ts, tz).strftime(str_format)
