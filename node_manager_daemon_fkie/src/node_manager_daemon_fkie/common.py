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

import rospy
import roslib
import rospkg
from xml.dom import minidom

MANIFEST_FILE = 'manifest.xml'
PACKAGE_FILE = 'package.xml'
EMPTY_PATTERN = re.compile('\b', re.I)
INCLUDE_PATTERN = ["\s*(\$\(find.*?)\"",
                   "file=\"(.*?)\"",
                   "textfile=\"(.*?)\"",
                   "binfile=\"(.*?)\"",
                   "\"\s*(pkg:\/\/.*?)\"",
                   "\"\s*(package:\/\/.*?)\""]
SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.nmprofile', '.sync', '.test', '.xml', '.xacro']

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
        try:
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
        except OSError:
            return (None, None)
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
                pkg = get_pkg_path(pkg_name)
                path_suffix = path[groups.end():].rstrip("'")
                if path_suffix.startswith('/'):
                    paths = roslib.packages.find_resource(pkg_name, path_suffix.strip(os.path.sep))
                    if len(paths) > 0:
                        # if more then one launch file is found, take the first one
                        return paths[0]
                if path_suffix:
                    return os.path.normpath(os.path.join(pkg, path_suffix))
                else:
                    return "%s%s" % (os.path.normpath(pkg), os.path.sep)
    if path.startswith('file://'):
        result = path[7:]
    return os.path.normpath(os.path.join(pwd, result))


def replace_paths(text, pwd='.'):
    '''
    Like meth:interpret_path(), but replaces all matches in the text and retain other text.
    '''
    result = text
    path_pattern = re.compile(r"(\$\(find .*?\)/)|(pkg:\/\/.*?/)|(package:\/\/.*?/)")
    for groups in path_pattern.finditer(text):
        for index in range(groups.lastindex):
            path = groups.groups()[index]
            if path:
                rpath = interpret_path(path, pwd)
                result = result.replace(path, rpath)
    return result


def replace_internal_args(content, resolve_args={}, path=None):
    '''
    Load the content with xml parser, search for arg-nodes and replace the arguments in whole content.
    :return: True if something was replaced, new content and detected arguments
    :rtype: (bool, str, {str: str})
    '''
    new_content = content
    replaced = False
    try:
        resolve_args_intern = {}
        for arg_key, args_val in resolve_args.items():
            replaced = True
            new_content = new_content.replace('$(arg %s)' % arg_key, args_val)
        xml_nodes = minidom.parseString(new_content).getElementsByTagName('launch')
        for node in xml_nodes:
            for child in node.childNodes:
                if child.localName == 'arg' and child.hasAttributes():
                    aname = ''
                    aval = ''
                    for argi in range(child.attributes.length):
                        arg_attr = child.attributes.item(argi)
                        if arg_attr.localName == 'name':
                            aname = arg_attr.value
                        elif arg_attr.localName in ['value', 'default']:
                            aval = arg_attr.value
                    if aname:
                        resolve_args_intern[aname] = aval
        for arg_key, args_val in resolve_args_intern.items():
            new_content = new_content.replace('$(arg %s)' % arg_key, args_val)
            replaced = True
    except Exception as err:
        print "%s in %s" % (utf8(err), path)
        rospy.logdebug("%s in %s" % (utf8(err), path))
    return replaced, new_content, resolve_args_intern


def replace_arg(value, resolve_args):
    # test for if statement
    re_if = re.compile(r"\$\(arg.(?P<name>.*?)\)")
    for arg in re_if.findall(value):
        if arg in resolve_args:
            return value.replace('$(arg %s)' % arg, resolve_args[arg])
    return value


def __get_include_args(content, resolve_args):
    included_files = []
    try:
        xml_nodes = minidom.parseString(content).getElementsByTagName('include')
        for node in xml_nodes:
            if node.nodeType == node.ELEMENT_NODE and node.hasAttributes():
                filename = ''
                for ai in range(node.attributes.length):
                    attr = node.attributes.item(ai)
                    if attr.localName == 'file':
                        filename = attr.value
                inc_args = node.getElementsByTagName('arg')
                resolved_inc_args = {}
                for inc_arg in inc_args:
                    if inc_arg.nodeType == node.ELEMENT_NODE and inc_arg.hasAttributes():
                        aname = ''
                        aval = ''
                        skip = False
                        for argi in range(inc_arg.attributes.length):
                            arg_attr = inc_arg.attributes.item(argi)
                            if arg_attr.localName == 'name':
                                aname = arg_attr.value
                            elif arg_attr.localName in ['value', 'default']:
                                aval = arg_attr.value
                            elif arg_attr.localName == 'if':
                                val = replace_arg(arg_attr.value, resolve_args)
                                skip = val in ['false', '0']
                            elif arg_attr.localName == 'unless':
                                val = replace_arg(arg_attr.value, resolve_args)
                                skip = val in ['true', '1']
                        if aname and not skip:
                            resolved_inc_args[aname] = aval
                if filename:
                    included_files.append((filename, resolved_inc_args))
    except Exception as err:
        print err
        rospy.logdebug(utf8(err))
    return included_files


def included_files(string,
                   recursive=True,
                   unique=False,
                   include_pattern=INCLUDE_PATTERN,
                   resolve_args={},
                   unique_files=[]):
    '''
    If the `string` parameter is a valid file the content of this file will be parsed.
    In other case the `string` is parsed to find included files.

    :param str string: Path to an exists file or test with included file.
    :param bool recursive: parse also found included files (Default: True)
    :param bool unique: returns the same files once (Default: False)
    :param include_pattern: the list with patterns to find include files.
    :type include_pattern: [str]
    :param resolve_args: dictionary with arguments to resolve arguments in path names
    :type resolve_args: {str, str}
    :return: Returns an iterator with tuple of given string, line number, path of included file and a dictionary with all defined arguments.
             if `unique` is True the line number is zero
    :rtype: iterator with (str, int, str, {str:str})
    '''
    re_filelist = EMPTY_PATTERN
    if include_pattern:
        # create regular expression from pattern
        re_filelist = re.compile(r"%s" % '|'.join(include_pattern))
    pwd = '.'
    content = string
    # read file content if file exists
    if os.path.exists(string) and not os.path.isdir(string):
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
    inc_files_forward_args = []
    # replace the arguments and detect arguments for include-statements
    if (string.endswith(".launch")):
        _replaced, content, _resolve_args_intern = replace_internal_args(content, path=string)
        inc_files_forward_args = __get_include_args(content, resolve_args)
    my_unique_files = unique_files
    if not unique_files:
        my_unique_files = list()
    # search for include pattern in the content without comments
    for groups in re_filelist.finditer(content):
        for index in range(groups.lastindex):
            filename = groups.groups()[index]
            if filename:
                try:
                    forward_args = {}
                    if inc_files_forward_args and inc_files_forward_args[0][0] == filename:
                        forward_args = inc_files_forward_args[0][1]
                        inc_files_forward_args.pop(0)
                    resolve_args_all = dict(resolve_args)
                    resolve_args_all.update(forward_args)
                    try:
                        # try to resolve path
                        filename = interpret_path(filename, pwd)
                    except Exception as err:
                        rospy.logwarn(utf8(err))
                    if os.path.isdir(filename):
                        filename = ''
                    if filename:
                        publish = not unique or (unique and filename not in my_unique_files)
                        if publish:
                            my_unique_files.append(filename)
                            # transform found position to line number
                            position = content.count("\n", 0, groups.start()) + 1
                            yield (string, position, filename, resolve_args_all)
                    # for recursive search
                    if os.path.isfile(filename):
                        if recursive:
                            ext = os.path.splitext(filename)
                            if ext[1] in SEARCH_IN_EXT:
                                for res_item in included_files(filename, recursive, unique, include_pattern, resolve_args_all):
                                    publish = not unique or (unique and res_item[2] not in my_unique_files)
                                    if publish:
                                        my_unique_files.append(res_item[2])
                                        yield res_item
                except Exception as e:
                    rospy.logwarn(utf8(e))
