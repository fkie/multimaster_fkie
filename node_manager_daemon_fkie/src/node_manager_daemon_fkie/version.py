import os
import roslib
import subprocess
import sys
import xml.dom.minidom as dom

from node_manager_daemon_fkie.supervised_popen import SupervisedPopen


def detect_version(package):
    '''
    Try to detect the current version from git, installed VERSION/DATE files or package.xml
    '''
    version = 'unknown'
    date = 'unknown'
    try:
        pkg_path = roslib.packages.get_pkg_dir(package)
        if pkg_path is not None and os.path.isfile("%s/VERSION" % pkg_path):
            try:
                with open("%s/VERSION" % pkg_path) as f:
                    version = f.read()
                    version = version.strip()
                with open("%s/DATE" % pkg_path) as f:
                    datetag = f.read().split()
                    if datetag:
                        date = datetag[0]
            except Exception as err:
                print >> sys.stderr, "version detection error: %s" % err
        elif os.path.isdir("%s/../.git" % pkg_path):
            try:
                os.chdir(pkg_path)
                ps = SupervisedPopen(['git', 'describe', '--tags', '--dirty', '--always', '--abbrev=8'], stdout=subprocess.PIPE)
                output = ps.stdout.read()
                version = output.strip()
                ps = SupervisedPopen(['git', 'show', '-s', '--format=%ci'], stdout=subprocess.PIPE)
                output = ps.stdout.read().split()
                if output:
                    date = output[0]
            except Exception as err:
                print >> sys.stderr, "version detection error: %s" % err
        else:
            ppath = roslib.packages.find_resource(package, 'package.xml')
            if ppath:
                doc = dom.parse(ppath[0])
                version_tags = doc.getElementsByTagName("version")
                if version_tags:
                    version = version_tags[0].firstChild.data
                    version = version
                else:
                    print >> sys.stderr, "version detection: no version tag in package.xml found!"
            else:
                print >> sys.stderr, "version detection: package.xml not found!"
    except Exception as err:
        print >> sys.stderr, "version detection error: %s" % err
    return version, date
