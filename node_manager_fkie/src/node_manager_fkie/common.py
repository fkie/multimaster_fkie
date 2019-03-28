import os
import rospy

import node_manager_fkie as nm

MANIFEST_FILE = 'manifest.xml'
PACKAGE_FILE = 'package.xml'

try:
    from catkin_pkg.package import parse_package
    CATKIN_SUPPORTED = True
except ImportError:
    CATKIN_SUPPORTED = False

PACKAGE_CACHE = {}


def get_ros_home():
    '''
    Returns the ROS HOME depending on ROS distribution API.

    :return: ROS HOME path
    :rtype: str
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


def lnamespace(name):
    '''
    Splits the given name into first part of the namespace and the rest. The rest
    contains no leading slash. The leading slash is own first part.

    :param str name: the name of the node or namespace.
    :return: A tuple of the first part of the namespace and the rest.
    :rtype: tuple(str, str)
    '''
    ns_list = name.split(rospy.names.SEP)
    if not ns_list[0]:
        return rospy.names.SEP, name.lstrip(rospy.names.SEP)
    if len(ns_list) == 1:
        return ns_list[0], ''
    return ns_list[0], name.replace('%s%s' % (ns_list[0], rospy.names.SEP), '')


def namespace(name):
    '''
    :param str name: the name of the node or namespace.
    :return: The namespace of given name. The last character is always a slash.
    :rtype: str
    '''
    result = os.path.dirname(name)
    if not result.endswith(rospy.names.SEP):
        result += rospy.names.SEP
    return result


def normns(name):
    '''
    Replaces double slashes by one slash.

    :param str name: the name of the node or namespace.
    :rtype: str
    '''
    sep = rospy.names.SEP
    result = name.replace('%s%s' % (sep, sep), sep)
    return result


def get_rosparam(param, masteruri):
    '''
    Get parameter using :class:`rospy.msproxy.MasterProxy`
    '''
    if masteruri:
        try:
            master = rospy.msproxy.MasterProxy(masteruri)
            return master[param]  # MasterProxy does all the magic for us
        except KeyError:
            return {}


def delete_rosparam(param, masteruri):
    '''
    Delete parameter using :class:`rospy.msproxy.MasterProxy`
    '''
    if masteruri:
        try:
            master = rospy.msproxy.MasterProxy(masteruri)
            del master[param]  # MasterProxy does all the magic for us
        except Exception:
            pass


def to_pkg(path):
    '''
    Searches the package name for given path and create an URL starting with pkg://

    :see: uses :meth:`package_name`
    '''
    result = path
    pkg, pth = package_name(os.path.dirname(path))
    if pkg is not None:
        result = "pkg://%s%s" % (pkg, path.replace(pth, ''))
    return result


def resolve_pkg(pkg, grpc_url):
    '''
    splits pkg url (pkg://package/launch) into package and launch file part and replace package by path.

    :rtype: str
    '''
    if pkg.startswith('pkg://'):
        url = pkg.replace('pkg://', '')
        splits = url.split(os.path.sep, 1)
        if len(splits) == 2:
            packages = nm.nmd().get_packages(grpc_url)
            for path, pkgname in packages.items():
                if pkgname == splits[0]:
                    return os.path.join(path, splits[1])
    raise Exception('invalid package url to split: %s' % pkg)


def package_name(path):
    '''
    Returns for given directory a tuple of package name and package path or None values.
    The results are cached!

    :rtype: tuple(name, path)
    '''
    return nm.nmd().package_name(path)
