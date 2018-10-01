import os
import rospy
from urlparse import urlparse
from master_discovery_fkie.common import get_hostname, get_port, masteruri_from_ros
from node_manager_daemon_fkie.common import get_nmd_url

import node_manager_fkie as nm

MANIFEST_FILE = 'manifest.xml'
PACKAGE_FILE = 'package.xml'

try:
    from catkin_pkg.package import parse_package
    CATKIN_SUPPORTED = True
except ImportError:
    CATKIN_SUPPORTED = False

PACKAGE_CACHE = {}


def utf8(s, errors='replace'):
    if isinstance(s, (str, buffer)):
        return unicode(s, "utf-8", errors=errors)
    elif not isinstance(s, unicode):
        return unicode(str(s))
    return s


def get_ros_home():
    '''
    Returns the ROS HOME depending on ROS distribution API.
    @return: ROS HOME path
    @rtype: C{str}
    '''
    try:
        import rospkg.distro
        distro = rospkg.distro.current_distro_codename()
        if distro in ['electric', 'diamondback', 'cturtle']:
            import roslib.rosenv
            return roslib.rosenv.get_ros_home()
        else:
            from rospkg import get_ros_home
            return get_ros_home()
    except Exception:
        from roslib import rosenv
        return rosenv.get_ros_home()


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


def resolve_paths(text):
    '''
    Searches in text for $(find ...) statements and replaces it by the package path.
    @return: text with replaced statements.
    '''
    result = text
    startIndex = text.find('$(')
    if startIndex > -1:
        endIndex = text.find(')', startIndex + 2)
        script = text[startIndex + 2: endIndex].split()
        if len(script) == 2 and (script[0] == 'find'):
            pkg = ''
            try:
                from rospkg import RosPack
                rp = RosPack()
                pkg = rp.get_path(script[1])
            except Exception:
                import roslib
                pkg = roslib.packages.get_pkg_dir(script[1])
            return result.replace(text[startIndex: endIndex + 1], pkg)
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
    Returns for given directory a tuple of package name and package path or None values.
    The results are cached!
    :rtype: tuple(name, path)
    '''
    return (nm.nmd().package_name(path), path)
#     if not (path is None) and path and path != os.path.sep and os.path.isdir(path):
#         if path in PACKAGE_CACHE:
#             return PACKAGE_CACHE[path]
#         package = os.path.basename(path)
#         fileList = os.listdir(path)
#         for f in fileList:
#             if f == MANIFEST_FILE:
#                 PACKAGE_CACHE[path] = (package, path)
#                 return (package, path)
#             if CATKIN_SUPPORTED and f == PACKAGE_FILE:
#                 try:
#                     pkg = parse_package(os.path.join(path, f))
#                     PACKAGE_CACHE[path] = (pkg.name, path)
#                     return (pkg.name, path)
#                 except Exception:
#                     return (None, None)
#         PACKAGE_CACHE[path] = package_name(os.path.dirname(path))
#         return PACKAGE_CACHE[path]
#     return (None, None)


def is_package(file_list):
    return (MANIFEST_FILE in file_list or (CATKIN_SUPPORTED and PACKAGE_FILE in file_list))


def grpc_url_from_path(grpc_path):
    url = grpc_path
    if not grpc_path:
        url = grpc_create_url('', '')
    if url and not url.startswith('grpc://'):
        raise ValueError("Invalid grpc path to split: %s; `grpc` scheme missed!" % grpc_path)
    url_parse_result = urlparse(url)
    return 'grpc://%s' % (url_parse_result.netloc)


def grpc_join(url, path):
#     import inspect
#     print("CALLERfmm:", inspect.stack()[1][3])
    if not path.startswith('grpc://'):
        if not url.startswith('grpc://'):
            return grpc_create_url(url, path)
        if path.startswith(os.path.sep):
            return '%s%s' % (url, path)
        return '%s%s%s' % (url, os.path.sep, path)
    return path


def grpc_create_url(masteruri, path):
#     import inspect
#     print("CALLERfmm:", inspect.stack()[1][3])
    if path.startswith(os.path.sep) or not path:
        return "%s%s" % (get_nmd_url(masteruri), path)
    return "%s%s%s" % (get_nmd_url(masteruri), os.path.sep, path)


def grpc_split_url(grpc_path, with_scheme=False):
    url = grpc_path
    if not grpc_path:
        url = get_nmd_url()
    if url and not url.startswith('grpc://'):
        raise ValueError("Invalid grpc path to split: %s; `grpc` scheme missed!" % grpc_path)
    url_parse_result = urlparse(url)
    if with_scheme:
        return ("%s://%s" % (url_parse_result.scheme, url_parse_result.netloc), url_parse_result.path)
    return (url_parse_result.netloc, url_parse_result.path)
