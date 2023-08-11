# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import unittest
from datetime import tzinfo, timedelta

from fkie_multimaster_pylib import formats

PKG = 'fkie_multimaster_msgs'


class TestFormatsLib(unittest.TestCase):
    '''
    '''

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_sizeof_fmt(self):
        sizeof_str = formats.sizeof_fmt(12345678)
        sizeof_str_res = "12MiB"
        self.assertEqual(sizeof_str_res, sizeof_str, "wrong sizeof_fmt, expected: %s, got: '%s'" % (
            sizeof_str_res, sizeof_str))

    def test_formated_ts(self):

        class UTC(tzinfo):
            def utcoffset(self, dt):
                return timedelta(0)

            def tzname(self, dt):
                return "UTC"

            def dst(self, dt):
                return timedelta(0)

        tsstr_ff = formats.timestamp_fmt(
            1557480759.608808, with_date=False, with_nanosecs=False, tz=UTC())
        tsstr_ff_res = "09:32:39"
        self.assertEqual(tsstr_ff_res, tsstr_ff, "wrong timestamp_fmt(value, False, False), expected: %s, got: %s" % (
            tsstr_ff_res, tsstr_ff))
        tsstr_tf = formats.timestamp_fmt(
            1557480759.608808, with_date=True, with_nanosecs=False, tz=UTC())
        tsstr_tf_res = "09:32:39 (10.05.2019)"
        self.assertEqual(tsstr_tf_res, tsstr_tf, "wrong timestamp_fmt(value, True, False), expected: %s, got: %s" % (
            tsstr_tf_res, tsstr_tf))
        tsstr_tt = formats.timestamp_fmt(
            1557480759.608808, with_date=True, with_nanosecs=True, tz=UTC())
        tsstr_tt_res = "09:32:39.608808 (10.05.2019)"
        self.assertEqual(tsstr_tt_res, tsstr_tt, "wrong timestamp_fmt(value, True, True), expected: %s, got: %s" % (
            tsstr_tt_res, tsstr_tt))
        tsstr_ft = formats.timestamp_fmt(
            1557480759.608808, with_date=False, with_nanosecs=True, tz=UTC())
        tsstr_ft_res = "09:32:39.608808"
        self.assertEqual(tsstr_ft_res, tsstr_ft, "wrong timestamp_fmt(value, False, True), expected: %s, got: %s" % (
            tsstr_ft_res, tsstr_ft))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, os.path.basename(__file__), TestFormatsLib)
