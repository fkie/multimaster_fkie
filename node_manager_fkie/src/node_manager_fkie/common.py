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
      from rospkg import get_ros_home
      return get_ros_home()
  except:
#    import traceback
#    print traceback.format_exc(1)
    from roslib import rosenv
    return rosenv.get_ros_home()


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

def resolve_paths(text):
  '''
  Searches in text for $(find ...) statements and replaces it by the package path.
  @return: text with replaced statements.
  '''
  result = text
  startIndex = text.find('$(')
  if startIndex > -1:
    endIndex = text.find(')', startIndex+2)
    script = text[startIndex+2:endIndex].split()
    if len(script) == 2 and (script[0] == 'find'):
      pkg = ''
      try:
        from rospkg import RosPack
        rp = RosPack()
        pkg = rp.get_path(script[1])
      except:
        import roslib
        pkg = roslib.packages.get_pkg_dir(script[1])
      return result.replace(text[startIndex:endIndex+1], pkg)
  return result

def package_name(path):
  '''
  Returns for given directory a tuple of package name and package path or None values.
  The results are cached!
  @rtype: C{(name, path)}
  '''
  if not (path is None) and path and path != os.path.sep and os.path.isdir(path):
    if PACKAGE_CACHE.has_key(path):
      return PACKAGE_CACHE[path]
    package = os.path.basename(path)
    fileList = os.listdir(path)
    for f in fileList:
      if f == MANIFEST_FILE:
        PACKAGE_CACHE[path] = (package, path)
        return (package, path)
      if CATKIN_SUPPORTED and f == PACKAGE_FILE:
        try:
          pkg = parse_package(os.path.join(path, f))
          PACKAGE_CACHE[path] = (pkg.name, path)
          return (pkg.name, path)
        except:
          return (None,None)
    PACKAGE_CACHE[path] = package_name(os.path.dirname(path))
    return PACKAGE_CACHE[path]
  return (None, None)

def is_package(file_list):
  return (MANIFEST_FILE in file_list or (CATKIN_SUPPORTED and PACKAGE_FILE in file_list))
