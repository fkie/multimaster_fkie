import os

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
    if 'manifest.xml' in fileList or 'package.xml' in fileList:
      return {os.path.basename(path) : path}
    for f in fileList:
      ret = get_packages(os.path.sep.join([path, f]))
      result = dict(ret.items() + result.items())
  return result

def package_name(dir):
  '''
  Returns for given directory a tuple of package name and package path or None values.
  @rtype: C{(name, path)}
  '''
  if not (dir is None) and dir and dir != os.path.sep and os.path.isdir(dir):
    package = os.path.basename(dir)
    fileList = os.listdir(dir)
    for file in fileList:
      if file == 'manifest.xml' or file == 'package.xml':
        return (package, dir)
    return package_name(os.path.dirname(dir))
  return (None, None)

def is_package(file_list):
  return ('manifest.xml' in file_list or 'package.xml' in file_list)
