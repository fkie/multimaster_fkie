import os
import roslib
import rospy

from master_discovery_fkie.common import resolve_url
from node_manager_fkie.common import utf8


def interpret_path(path):
    '''
    Tries to determine the path of the included file. The statement of
    C{$(find 'package')} will be resolved.
    @param path: the sting which contains the included path
    @type path: C{str}
    @return: C{$(find 'package')} will be resolved. The prefixes `file:///`, `package://` or `pkg://` are also resolved.
    Otherwise the parameter itself normalized by `os.path.normpath` will be returned.
    @rtype: C{str}
    '''
    path = path.strip()
    index = path.find('$')
    if index > -1:
        startIndex = path.find('(', index)
        if startIndex > -1:
            endIndex = path.find(')', startIndex + 1)
            script = path[startIndex + 1:endIndex].split()
            if len(script) == 2 and (script[0] == 'find'):
                try:
                    pkg = roslib.packages.get_pkg_dir(script[1])
                    return os.path.normpath('%s/%s' % (pkg, path[endIndex + 1:]))
                except Exception as e:
                    rospy.logwarn(utf8(e))
    else:
        try:
            return resolve_url(path)
        except ValueError, e:
            pass
    return os.path.normpath(path)
