# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Fraunhofer FKIE/CMS, Alexander Tiderko
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
import roslib
import subprocess
import sys
import xml.dom.minidom as dom

from fkie_node_manager_daemon.common import utf8
from fkie_node_manager_daemon.supervised_popen import SupervisedPopen

VERSION = 'unknown'
DATE = 'unknown'


def detect_version(package):
    '''
    Try to detect the current version from git, installed VERSION/DATE files or package.xml
    '''
    global VERSION
    global DATE
    if VERSION != 'unknown':
        return VERSION, DATE
    version = 'unknown'
    date = 'unknown'
    try:
        pkg_path = roslib.packages.get_pkg_dir(package)
        if pkg_path is not None and os.path.isfile("%s/VERSION" % pkg_path):
            try:
                with open("%s/VERSION" % pkg_path) as f:
                    version = f.read()
                    version = version.strip().decode('utf-8')
                with open("%s/DATE" % pkg_path) as f:
                    datetag = f.read().split()
                    if datetag:
                        date = datetag[0].decode('utf-8')
            except Exception as err:
                sys.stderr.write("version detection error: %s" % utf8(err))
        elif os.path.isdir("%s/../.git" % pkg_path):
            try:
                os.chdir(pkg_path)
                ps = SupervisedPopen(['git', 'describe', '--tags', '--dirty', '--always', '--abbrev=8'], stdout=subprocess.PIPE, object_id='get git version')
                output = ps.stdout.read().decode('utf-8')
                version = output.strip()
                ps = SupervisedPopen(['git', 'show', '-s', '--format=%ci'], stdout=subprocess.PIPE, object_id='get git date')
                output = ps.stdout.read().split()
                if output:
                    date = output[0].decode('utf-8')
            except Exception as err:
                sys.stderr.write("version detection error: %s" % utf8(err))
        else:
            ppath = roslib.packages.find_resource(package, 'package.xml')
            if ppath:
                doc = dom.parse(ppath[0])
                version_tags = doc.getElementsByTagName("version")
                if version_tags:
                    version = version_tags[0].firstChild.data
                    version = version
                else:
                    sys.stderr.write("version detection: no version tag in package.xml found!")
            else:
                sys.stderr.write("version detection: package.xml not found!")
    except Exception as err:
        sys.stderr.write("version detection error: %s" % utf8(err))
    VERSION = version
    DATE = date
    return version, date
