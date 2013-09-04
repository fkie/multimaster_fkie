import os

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
      import rospkg
      return rospkg.get_ros_home()
  except:
#    import traceback
#    print traceback.format_exc()
    import roslib.rosenv
    return roslib.rosenv.get_ros_home()


def masteruri_from_ros():
  '''
  Returns the master URI depending on ROS distribution API.
  @return: ROS master URI
  @rtype: C{str}
  '''
  try:
    import rospkg.distro
    distro = rospkg.distro.current_distro_codename()
    if distro in ['electric', 'diamondback', 'cturtle']:
      import roslib.rosenv
      return roslib.rosenv.get_master_uri()
    else:
      import rosgraph
      return rosgraph.rosenv.get_master_uri()
  except:
    return os.environ['ROS_MASTER_URI']

def get_packages(path):
  result = {}
  if os.path.isdir(path):
    fileList = os.listdir(path)
    if MANIFEST_FILE in fileList:
      return {os.path.basename(path) : path}
    if CATKIN_SUPPORTED and PACKAGE_FILE in fileList:
      try:
        pkg = parse_package(path)
        return {pkg.name : path}
      except:
        pass
      return {}
    for f in fileList:
      ret = get_packages(os.path.join(path, f))
      result = dict(ret.items() + result.items())
  return result

def package_name(dir):
  '''
  Returns for given directory a tuple of package name and package path or None values.
  The results are cached!
  @rtype: C{(name, path)}
  '''
  if not (dir is None) and dir and dir != os.path.sep and os.path.isdir(dir):
    if PACKAGE_CACHE.has_key(dir):
      return PACKAGE_CACHE[dir]
    package = os.path.basename(dir)
    fileList = os.listdir(dir)
    for file in fileList:
      if file == MANIFEST_FILE:
        PACKAGE_CACHE[dir] = (package, dir)
        return (package, dir)
      if CATKIN_SUPPORTED and file == PACKAGE_FILE:
        try:
          pkg = parse_package(os.path.join(dir, file))
          PACKAGE_CACHE[dir] = (pkg.name, dir)
          return (pkg.name, dir)
        except:
          return (None,None)
    PACKAGE_CACHE[dir] = package_name(os.path.dirname(dir))
    return PACKAGE_CACHE[dir]
  return (None, None)

def is_package(file_list):
  return (MANIFEST_FILE in file_list or (CATKIN_SUPPORTED and PACKAGE_FILE in file_list))
