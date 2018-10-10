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
import re
from urlparse import urlparse
from master_discovery_fkie.common import masteruri_from_master

import rospy
import roslib
import rospkg

MANIFEST_FILE = 'manifest.xml'
PACKAGE_FILE = 'package.xml'
EMPTY_PATTERN = re.compile('\b', re.I)
INCLUDE_PATTERN = ["\s*(\$\(find.*?)\"",
                   "file=\"(.*?)\"",
                   "textfile=\"(.*?)\"",
                   "binfile=\"(.*?)\"",
                   "\"\s*(pkg:\/\/.*?)\"",
                   "\"\s*(package:\/\/.*?)\""]
SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.nmprofile', '.sync', '.test', '.xml']

NMD_SERVER_PORT_OFFSET = 1010

try:
    from catkin_pkg.package import parse_package
    CATKIN_SUPPORTED = True
except ImportError:
    CATKIN_SUPPORTED = False

_get_pkg_path_var = None


PACKAGE_CACHE = {}


def utf8(s, errors='replace'):
    if isinstance(s, (str, buffer)):
        return unicode(s, "utf-8", errors=errors)
    elif not isinstance(s, unicode):
        return unicode(str(s))
    return s


def get_cwd(cwd, binary=''):
    result = ''
    if cwd == 'node':
        result = os.path.dirname(binary)
    elif cwd == 'cwd':
        result = os.getcwd()
    elif cwd == 'ros-root':
        result = rospkg.get_ros_root()
    else:
        result = rospkg.get_ros_home()
    if not os.path.exists(result):
        try:
            os.makedirs(result)
        except OSError:
            # exist_ok=True
            pass
    return result


def equal_uri(url1, url2):
    return url1.rstrip(os.path.sep) == url2.rstrip(os.path.sep)


def get_nmd_url(uri='', prefix='grpc://'):
    muri = uri
    if not muri:
        muri = masteruri_from_master()
    o = urlparse(muri)
    port = o.port
    if o.scheme == 'http':
        port += NMD_SERVER_PORT_OFFSET
    return "%s%s:%d" % (prefix, o.hostname, port)


def get_masteruri_from_nmd(grpc_path):
    if not grpc_path:
        return masteruri_from_master()
    if not grpc_path.startswith('grpc://'):
        raise ValueError("Invalid grpc path to get masteruri: %s; `grpc` scheme missed!" % grpc_path)
    o = urlparse(grpc_path)
    port = o.port
    if o.scheme == 'grpc':
        port -= NMD_SERVER_PORT_OFFSET
    return "http://%s:%d" % (o.hostname, port)


def get_nmd_port(uri=''):
    muri = uri
    if not muri:
        muri = masteruri_from_master()
    o = urlparse(muri)
    port = o.port
    if o.scheme == 'http':
        port += NMD_SERVER_PORT_OFFSET
    return port


def get_rosparam(param, masteruri):
    if masteruri:
        try:
            master = rospy.msproxy.MasterProxy(masteruri)
            return master[param]  # MasterProxy does all the magic for us
        except KeyError:
            return {}


def delete_rosparam(param, masteruri):
    if masteruri:
        try:
            master = rospy.msproxy.MasterProxy(masteruri)
            del master[param]  # MasterProxy does all the magic for us
        except Exception:
            pass


def get_packages(path):
    result = {}
    if os.path.isdir(path):
        fileList = os.listdir(path)
        if MANIFEST_FILE in fileList:
            return {os.path.basename(path): path}
        if CATKIN_SUPPORTED and PACKAGE_FILE in fileList:
            try:
                pkg = parse_package(path)
                return {pkg.name: path}
            except Exception:
                pass
            return {}
        for f in fileList:
            ret = get_packages(os.path.join(path, f))
            result = dict(ret.items() + result.items())
    return result


def to_url(path):
    '''
    Searches the package name for given path and create an URL starting with pkg://
    '''
    result = path
    pkg, pth = package_name(os.path.dirname(path))
    if pkg is not None:
        result = "pkg://%s%s" % (pkg, path.replace(pth, ''))
    return result


def package_name(path):
    '''
    The results are cached!
    :return: Returns for given directory a tuple of package name and package path.
    :rtype: tuple(str, str) or tuple(None, None)
    '''
    if path is not None and path and path != os.path.sep:
        dir_path = path
        if not os.path.isdir(dir_path):
            dir_path = os.path.dirname(dir_path)
        if dir_path in PACKAGE_CACHE:
            return PACKAGE_CACHE[dir_path]
        package = os.path.basename(dir_path)
        fileList = os.listdir(dir_path)
        for f in fileList:
            if f == MANIFEST_FILE:
                PACKAGE_CACHE[dir_path] = (package, dir_path)
                return (package, dir_path)
            if CATKIN_SUPPORTED and f == PACKAGE_FILE:
                try:
                    pkg = parse_package(os.path.join(dir_path, f))
                    PACKAGE_CACHE[dir_path] = (pkg.name, dir_path)
                    return (pkg.name, dir_path)
                except Exception:
                    return (None, None)
        PACKAGE_CACHE[dir_path] = package_name(os.path.dirname(dir_path))
        return PACKAGE_CACHE[dir_path]
    return (None, None)


def is_package(file_list):
    return (MANIFEST_FILE in file_list or (CATKIN_SUPPORTED and PACKAGE_FILE in file_list))


def get_pkg_path(package_name):
    ''' :noindex: '''
    global _get_pkg_path_var
    if _get_pkg_path_var is None:
        try:
            import rospkg
            rp = rospkg.RosPack()
            _get_pkg_path_var = rp.get_path
        except ImportError:
            _get_pkg_path_var = roslib.packages.get_pkg_dir
    return _get_pkg_path_var(package_name)


def interpret_path(path, pwd='.'):
    '''
    Tries to determine the path of included file. The statement of $(find 'package') will be resolved.
    :param str path: the sting which contains the included path
    :param str pwd: current working path
    :return: `$(find 'package')` will be resolved. The prefixes `file://`, `package://` or `pkg://` are also resolved.
        Otherwise the parameter itself normalized by :py:func:`os.path.normpath` will be returned.
    :rtype: str
    '''
    result = path.strip()
    # try replace package name by package path
    pkg_pattern = re.compile(r"\$\(find (.*?)\)/|pkg:\/\/(.*?)/|package:\/\/(.*?)/")
    for groups in pkg_pattern.finditer(path):
        for index in range(groups.lastindex):
            pkg_name = groups.groups()[index]
            if pkg_name:
                pkg = _get_pkg_path(pkg_name)
                path_suffix = path[groups.end():]
                if path_suffix.startswith('/'):
                    paths = roslib.packages.find_resource(pkg_name, path_suffix.strip(os.path.sep))
                    if len(paths) > 0:
                        # if more then one launch file is found, take the first one
                        return paths[0]
                return os.path.normpath(os.path.join(pkg, path_suffix))
    if path.startswith('file://'):
        result = path[7:]
    return os.path.normpath(os.path.join(pwd, result))


def included_files(string,
                   recursive=True,
                   unique=False,
                   include_pattern=INCLUDE_PATTERN):
    '''
    If the `string` parameter is a valid file the content of this file will be parsed.
    In other case the `string` is parsed to find included files.
    :param str string: Path to an exists file or test with included file.
    :param bool recursive: parse also found included files (Default: True)
    :param bool unique: returns the same files once (Default: False)
    :param include_pattern: the list with patterns to find include files.
    :type include_pattern: [str]
    :return: Returns a set of included file is `unique` is True,
        otherwise a list of tuples with line number, path of included file and a recursive list of tuples with included files.
    :rtype: set() or [(int, str, [])]
    '''
    re_filelist = EMPTY_PATTERN
    if include_pattern:
        # create regular expression from pattern
        re_filelist = re.compile(r"%s" % '|'.join(include_pattern))
    result = []
    pwd = '.'
    content = string
    # read file content if file exists
    if os.path.exists(string):
        pwd = os.path.dirname(string)
        with open(string, 'r') as f:
            content = f.read()
            # replace XML comments by the same count of NEWLINES
            comment_pattern = re.compile(r"<!--.*?-->", re.DOTALL)
            match = comment_pattern.search(content)
            while match is not None:
                count_nl = content[match.start():match.end()].count('\n')
                content = content[:match.start()] + '\n' * count_nl + content[match.end():]
                match = comment_pattern.search(content, match.start())
    # search for include pattern in the content without comments
    for groups in re_filelist.finditer(content):
        for index in range(groups.lastindex):
            file_name = groups.groups()[index]
            if file_name:
                recursive_list = []
                try:
                    file_name = interpret_path(file_name, pwd)
                    # for recursive search
                    if recursive and os.path.isfile(file_name):
                        ext = os.path.splitext(file_name)
                        if ext[1] in SEARCH_IN_EXT:
                            res_list = included_files(file_name, recursive, unique, include_pattern)
                            if not unique:
                                # if not unique the result build a tree with all included files
                                recursive_list = res_list
                            else:
                                result += res_list
                except Exception as e:
                    rospy.logwarn(utf8(e))
                if not unique:
                    # transform found position to line number
                    result.append((content.count("\n", 0, groups.start()) + 1, file_name, recursive_list))
                else:
                    result.append(file_name)
                continue
    if unique:
        return set(result)
    return result
