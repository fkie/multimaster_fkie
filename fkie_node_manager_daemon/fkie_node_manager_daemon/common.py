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

from typing import Dict
from typing import List
from typing import Text
from typing import Tuple

from datetime import datetime
import os
import re
import sys

from ament_index_python import get_packages_with_prefixes
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from rclpy.node import Node
from xml.dom import minidom

SEP = '/'
PRIV_NAME = '~'
PACKAGE_FILE = 'package.xml'
EMPTY_PATTERN = re.compile(r'\b', re.I)
INCLUDE_PATTERN = [r"\s*(\$\(find-pkg-share.*?\)[^ \"]*)",
                   r"file=\"(.*?)\"",
                   r"textfile=\"(.*?)\"",
                   r"binfile=\"(.*?)\"",
                   r"\"\s*(pkg:\/\/.*?)\"",
                   r"\"\s*(package:\/\/.*?)\""]
SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg',
                 '.iface', '.nmprofile', '.sync', '.test', '.xml', '.xacro']

try:
    from catkin_pkg.package import parse_package
    CATKIN_SUPPORTED = True
except ImportError:
    CATKIN_SUPPORTED = False

_get_pkg_path_var = None


PACKAGE_CACHE = {}
SOURCE_PATH_TO_PACKAGES = {}


class IncludedFile():

    def __init__(self, path_or_str, line_number, inc_path, exists, raw_inc_path, rec_depth, args, size=0):
        '''
        Representation of an included file found in given string or path of a file.

        :param str path_or_str: path of file or content where to search. If it is a path, the content will be read from file.
        :param int line_number: line number of the occurrence. If `unique` is True the line number is zero.
        :param str inc_path: resolved path.
        :param bool exists: True if resolved path exists.
        :param str raw_inc_path: representation of included file without resolved arg and find statements.
        :param int rec_depth: depth of recursion. if `unique` is True the depth is zero
        :param dict(str:str) args: a dictionary with arguments forwarded within include tag for 'inc_path'.
        '''
        self.path_or_str = path_or_str
        self.line_number = line_number
        self.inc_path = inc_path
        self.exists = exists
        self.raw_inc_path = raw_inc_path
        self.rec_depth = rec_depth
        self.args = args
        self.size = size
        self.unset_default_args = {}

    def __repr__(self):
        result = "<IncludedFile "
        result += " from=%s" % self.path_or_str
        result += " line_number=%d" % self.line_number
        result += " inc_path=%s" % self.inc_path
        result += " raw_inc_path=%s" % self.raw_inc_path
        result += " exists=%s" % self.exists
        result += " size=%d" % self.size
        result += " rec_depth=%d" % self.rec_depth
        result += " args=%s" % self.args
        result += " />"
        return result


# def utf8(s, errors='replace'):
#     '''
#     Converts string to unicode.
#     '''
#     if sys.version_info <= (3, 0):
#         if isinstance(s, (str, buffer)):
#             return unicode(s, "utf-8", errors=errors)
#         elif not isinstance(s, unicode):
#             return unicode(str(s))
#     return s

def get_namespace(name: Text, with_sep_suffix: bool = True) -> Text:
    """
    Get the namespace of name. The namespace is returned with a
    trailing slash in order to favor easy concatenation and easier use
    within the global context.

    :param str name: name to return the namespace of. Must be a legal
        name. NOTE: an empty name will return the global namespace.
    :return str: Namespace of name. For example, '/wg/node1' returns '/wg/'. The
        global namespace is '/'. 
    :rtype: str
    :raise ValueError: if name is invalid
    """
    if name is None:
        raise ValueError('name')
    if not name:
        return SEP
    elif name[-1] == SEP:
        name = name[:-1]
    offset = 1 if with_sep_suffix else 0
    return name[:name.rfind(SEP)+offset] or SEP


def get_cwd(cwd: Text, binary: Text = '') -> Text:
    result = ''
    if cwd == 'node':
        result = os.path.dirname(binary)
    elif cwd == 'cwd':
        result = os.getcwd()
    # elif cwd == 'ros-root':
    #    result = rospkg.get_ros_root()
    # else:
    #    result = rospkg.get_ros_home()
    if result and not os.path.exists(result):
        try:
            os.makedirs(result)
        except OSError:
            # exist_ok=True
            pass
    return result


def sizeof_fmt(num: [float, int], suffix: str = 'B') -> Text:
    for unit in ['', 'Ki', 'Mi', 'Gi', 'Ti', 'Pi', 'Ei', 'Zi']:
        if abs(num) < 1024.0:
            return '%.0f%s%s' % (num, unit, suffix)
        num /= 1024.0
    return '%.0f%s%s' % (num, 'YiB', suffix)


def formated_ts(stamp: float, with_date: bool = True, with_nanosecs: bool = True, tz=None) -> Text:
    ts = stamp
    if hasattr(stamp, 'secs'):
        ts = stamp.secs + stamp.secs / 1000000000.
    str_format = '%H:%M:%S'
    if with_nanosecs:
        str_format += '.%f'
    if with_date:
        str_format += ' (%d.%m.%Y)'
    return datetime.fromtimestamp(ts, tz).strftime(str_format)


def get_packages(path: [Text, None]) -> Dict[Text, Text]:
    result = {}
    if path is None:
        # we use ament to get the list of all packages
        return get_packages_with_prefixes()
    # TODO: this is old style code
    if os.path.isdir(path):
        fileList = os.listdir(path)
        if CATKIN_SUPPORTED and PACKAGE_FILE in fileList:
            try:
                pkg = parse_package(path)
                return {pkg.name: path}
            except Exception:
                pass
            return {}
        for f in fileList:
            ret = get_packages(os.path.join(path, f))
            result.update(ret)
    return result


def package_name(path: Text) -> Tuple[Text, Text]:
    '''
    The results are cached!

    :return: Returns for given directory a tuple of package name and package path.
    :rtype: tuple(str, str), empty strings if no package was found
    '''
    if path and path != os.path.sep:
        dir_path = path
        if not os.path.isdir(dir_path):
            dir_path = os.path.dirname(dir_path)
        if dir_path in PACKAGE_CACHE:
            return PACKAGE_CACHE[dir_path]
        try:
            fileList = os.listdir(dir_path)
            if CATKIN_SUPPORTED and PACKAGE_FILE in fileList:
                try:
                    pkg = parse_package(os.path.join(
                        dir_path, os.path.join(dir_path, PACKAGE_FILE)))
                    PACKAGE_CACHE[dir_path] = (pkg.name, dir_path)
                    return (pkg.name, dir_path)
                except Exception:
                    return ('', '')
            dir_path = os.path.dirname(dir_path)
            pname, pdir = package_name(dir_path)
            if pname:
                PACKAGE_CACHE[dir_path] = (pname, pdir)
                return (pname, pdir)
        except OSError:
            return ('', '')
    PACKAGE_CACHE[path] = ('', '')
    return ('', '')


def is_package(file_list: List[Text]) -> bool:
    return CATKIN_SUPPORTED and PACKAGE_FILE in file_list


def get_pkg_path(pkg_name: Text) -> Text:
    ''' :noindex: '''
    from ament_index_python import get_resource
    _, package_path = get_resource('packages', pkg_name)
    return package_path
    # old style:
    global _get_pkg_path_var
    if _get_pkg_path_var is None:
        rp = rospkg.RosPack()
        _get_pkg_path_var = rp.get_path
    return _get_pkg_path_var(pkg_name)


def reset_package_cache():
    global _get_pkg_path_var
    _get_pkg_path_var = None
    global PACKAGE_CACHE
    PACKAGE_CACHE = {}
    global SOURCE_PATH_TO_PACKAGES
    SOURCE_PATH_TO_PACKAGES = {}


def get_share_files_path_from_package(package_name, file_name):
    """
    Return the full path to a file in the share directory of a package.

    :raises: PackageNotFoundError if package is not found
    :raises: FileNotFoundError if the file is not found in the package
    :raises: MultipleLaunchFilesError if the file is found in multiple places
    """
    package_share_directory = get_package_share_directory(package_name)
    matching_file_paths = []
    for root, _dirs, files in os.walk(package_share_directory):
        for name in files:
            if name == file_name:
                matching_file_paths.append(os.path.join(root, name))
    return matching_file_paths


def interpret_path(path: Text, *, pwd: Text = '.', rosnode: [Node, None] = None) -> Text:
    '''
    Tries to determine the path of included file. The statement of $(find-pkg-share 'package') will be resolved.

    :param str path: the sting which contains the included path
    :param str pwd: current working path
    :return: `$(find-pkg-share 'package')` will be resolved. The prefixes `file://`, `package://` or `pkg://` are also resolved.
             Otherwise the parameter itself normalized by :py:func:`os.path.normpath` will be returned.
    :rtype: str
    '''
    result = path.strip()
    # try replace package name by package path
    pkg_pattern = re.compile(
        r"\$\(find-pkg-share (.*?)\)/|pkg:\/\/(.*?)/|package:\/\/(.*?)/")
    for groups in pkg_pattern.finditer(path):
        for index in range(groups.lastindex):
            pkg_name = groups.groups()[index]
            if pkg_name:
                pkg = get_package_share_directory(pkg_name)
                if rosnode is not None:
                    rosnode.get_logger().debug("rospkg.RosPack.get_path for '%s': %s" % (pkg_name, pkg))
                path_suffix = path[groups.end():].rstrip("'")
                #package_share_directory = pkg
                if path_suffix.startswith('/'):
                    matching_file_paths = get_share_files_path_from_package(
                        pkg_name, path_suffix.strip(os.path.sep))
                    if rosnode is not None:
                        rosnode.get_logger().debug(" search for resource with roslib.packages._find_resource, suffix '%s': %s" % (
                            path_suffix.strip(os.path.sep), matching_file_paths))
                    if len(matching_file_paths) > 0:
                        # if more then one launch file is found, take the first one
                        return matching_file_paths[0]
                full_path = os.path.normpath(os.path.join(pkg, path_suffix))
                #raise Exception("full_path: %s" % full_path)
                if path_suffix:
                    if not os.path.exists(full_path):
                        # we try to find the specific path in share
                        try:
                            paths = get_share_files_path_from_package(
                                pkg_name, path_suffix.strip(os.path.sep))
                            if paths:
                                return paths[0]
                        except Exception:
                            import traceback
                            if rosnode is not None:
                                rosnode.get_logger().warn("search in share space failed: %s" %
                                                          traceback.format_exc())
                    return full_path
                else:
                    return "%s%s" % (os.path.normpath(pkg), os.path.sep)
    if path.startswith('file://'):
        result = path[7:]
    return os.path.normpath(os.path.join(pwd, result))


def replace_paths(text: Text, pwd: Text = '.'):
    '''
    Like meth:interpret_path(), but replaces all matches in the text and retain other text.
    '''
    result = text
    path_pattern = re.compile(
        r"(\$\(find-pkg-share .*?\)/)|(pkg:\/\/.*?/)|(package:\/\/.*?/)")
    for groups in path_pattern.finditer(text):
        for index in range(groups.lastindex):
            path = groups.groups()[index]
            if path:
                rpath = interpret_path(path, pwd=pwd)
                result = result.replace(path, rpath)
    return result


def get_internal_args(content: Text, path: Text = None, only_default: bool = False, rosnode: [Node, None] = None):
    '''
    Load the content with xml parser, search for arg-nodes.
    :return: a dictionary with detected arguments
    :rtype: {str: str}
    '''
    new_content = content
    try:
        resolve_args_intern = {}
        xml_nodes = minidom.parseString(
            new_content).getElementsByTagName('launch')
        for node in xml_nodes:
            for child in node.childNodes:
                if child.localName == 'arg' and child.hasAttributes():
                    aname = ''
                    aval = ''
                    add_arg = True
                    for argi in range(child.attributes.length):
                        arg_attr = child.attributes.item(argi)
                        if arg_attr.localName == 'name':
                            aname = arg_attr.value
                        elif arg_attr.localName in ['value', 'default']:
                            aval = arg_attr.value
                            # do not add this argument to the result list if value is set and 'only_default' is True
                            if only_default and arg_attr.localName == 'value':
                                add_arg = False
                    if aname and add_arg:
                        resolve_args_intern[aname] = aval
    except Exception as err:
        print("%s while get_internal_args %s" % (err, path))
        if rosnode:
            rosnode.get_logger().debug("%s while get_internal_args %s" % (err, path))
    return resolve_args_intern


def replace_internal_args(content: Text, resolve_args: Dict[Text, Text] = {}, path: Text = None, rosnode: [Node, None] = None):
    '''
    Load the content with xml parser, search for arg-nodes and replace the arguments in whole content.
    :return: True if something was replaced, new content and detected arguments
    :rtype: (bool, str, {str: str})
    '''
    new_content = content
    replaced = False
    resolve_args_intern = {}
    try:
        for arg_key, args_val in resolve_args.items():
            replaced = True
            new_content = new_content.replace('$(arg %s)' % arg_key, args_val)
        resolve_args_intern = get_internal_args(content)
        for arg_key, args_val in resolve_args_intern.items():
            new_content = new_content.replace('$(arg %s)' % arg_key, args_val)
            replaced = True
    except Exception as err:
        print("%s in %s" % (err, path))
        if rosnode:
            rosnode.get_logger().debug("%s in %s" % (err, path))
    return replaced, new_content, resolve_args_intern


def get_arg_names(value):
    '''
    Searches for $(arg <name>) statements and returns a list with <name>.
    :rtype: [str]
    '''
    result = []
    re_if = re.compile(r"\$\(arg.(?P<name>.*?)\)")
    for arg in re_if.findall(value):
        result.append(arg)
    return result


def replace_arg(value, resolve_args: Dict[Text, Text]):
    # test for if statement
    result = value
    re_if = re.compile(r"\$\(arg.(?P<name>.*?)\)")
    for arg in re_if.findall(value):
        if arg in resolve_args:
            result = result.replace('$(arg %s)' % arg, resolve_args[arg])
    return result


def __get_include_args(content: Text, resolve_args: Dict[Text, Text], rosnode: [Node, None] = None):
    included_files = []
    try:
        xml_nodes = minidom.parseString(
            content).getElementsByTagName('include')
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
        print("__get_include_args reports: %s" % err)
        if rosnode:
            rosnode.get_logger().debug(err)
    return included_files


def find_included_files(string: Text,
                        *,
                        recursive: bool = True,
                        unique: bool = False,
                        include_pattern: List[Text] = INCLUDE_PATTERN,
                        search_in_ext: List[Text] = SEARCH_IN_EXT,
                        resolve_args: Dict[Text, Text] = {},
                        unique_files: List[Text] = [],
                        rec_depth: int = 0,
                        rosnode: [Node, None] = None):
    '''
    If the `string` parameter is a valid file the content of this file will be parsed.
    In other case the `string` is parsed to find included files.

    :param str string: Path to an exists file or test with included file.
    :param bool recursive: parse also found included files (Default: True)
    :param bool unique: returns the same files once (Default: False)
    :param include_pattern: the list with patterns to find include files.
    :type include_pattern: [str]
    :param search_in_ext: file extensions to search in
    :type search_in_ext: [str]
    :param resolve_args: dictionary with arguments to resolve arguments in path names
    :type resolve_args: {str, str}
    :return: Returns an iterator with IncludedFile-class
    :rtype: iterator with IncludedFile
    '''
    re_filelist = EMPTY_PATTERN
    if include_pattern:
        # create regular expression from pattern
        re_filelist = re.compile(r"%s" % '|'.join(include_pattern))
    pwd = '.'
    content = string
    content_info = 'content'
    # read file content if file exists
    if os.path.exists(string) and not os.path.isdir(string):
        pwd = os.path.dirname(string)
        content_info = string
        with open(string, 'r') as f:
            content = f.read()
            # replace XML comments by the same count of NEWLINES
            comment_pattern = re.compile(r"<!--.*?-->", re.DOTALL)
            match = comment_pattern.search(content)
            while match is not None:
                count_nl = content[match.start():match.end()].count('\n')
                content = content[:match.start()] + '\n' * \
                    count_nl + content[match.end():]
                match = comment_pattern.search(content, match.start())
    inc_files_forward_args = []
    # replace the arguments and detect arguments for include-statements
    resolve_args_intern = {}
    if (string.endswith('.launch') or string.find('.launch.') > 0):
        _replaced, content_resolved, resolve_args_intern = replace_internal_args(
            content, resolve_args=resolve_args, path=string)
        # intern args use only internal
        inc_files_forward_args = __get_include_args(
            content_resolved, resolve_args)
    my_unique_files = unique_files
    if not unique_files:
        my_unique_files = list()
    # search for include pattern in the content without comments
    for groups in re_filelist.finditer(content):
        if groups.lastindex is None:
            continue
        for index in range(groups.lastindex):
            filename = groups.groups()[index]
            rawname = filename
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
                        filename = replace_arg(filename, resolve_args_all)
                        filename = replace_arg(filename, resolve_args_intern)
                        filename = interpret_path(
                            filename, pwd=pwd, rosnode=rosnode)
                    except Exception as err:
                        if rosnode:
                            rosnode.get_logger().warn("Interpret file failed: %s" % err)
                    if os.path.isdir(filename):
                        filename = ''
                    exists = os.path.isfile(filename)
                    if filename:
                        publish = not unique or (
                            unique and filename not in my_unique_files)
                        if publish:
                            my_unique_files.append(filename)
                            # transform found position to line number
                            position = content.count(
                                "\n", 0, groups.start()) + 1
                            yield IncludedFile(string, position, filename, exists, rawname, rec_depth, forward_args)
                    # for recursive search
                    if exists:
                        if recursive:
                            try:
                                ext = os.path.splitext(filename)
                                if ext[1] in search_in_ext:
                                    for res_item in find_included_files(filename, recursive=recursive, unique=False, include_pattern=include_pattern, search_in_ext=search_in_ext, resolve_args=resolve_args_all, rec_depth=rec_depth+1):
                                        publish = not unique or (
                                            unique and res_item.inc_path not in my_unique_files)
                                        if publish:
                                            my_unique_files.append(
                                                res_item.inc_path)
                                            yield res_item
                            except Exception as e:
                                if rosnode:
                                    rosnode.get_logger().warn(
                                        "Error while recursive search for include pattern in %s: %s" % (filename, e))
                except Exception as e:
                    import traceback
                    if rosnode:
                        rosnode.get_logger().warn("Error while parse %s for include pattern: %s" %
                                                  (content_info, traceback.format_exc()))
