# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
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

import re
import os

import roslib; roslib.load_manifest('master_discovery_fkie')
import roslib.names
import rospy

EMPTY_PATTERN = re.compile('\b', re.I)

def masteruri_from_ros():
  '''
  Returns the master URI depending on ROS distribution API.
  
  :return: ROS master URI
  
  :rtype: str
  
  :see: rosgraph.rosenv.get_master_uri() (fuerte)
  
  :see: roslib.rosenv.get_master_uri() (prior)
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

def resolve_url(interface_url):
  '''
  The supported URL begins with `file:///`, `package://` or `pkg://`.
  The package URL will be resolved to a valid file path. If the file is in a
  subdirectory, you can replace the subdirectory by `///`. 
  
  E.g.: `package://master_discovery_fkie///master_discovery.launch`
  
  :raise ValueError: on invalid URL or not existent file
  
  :return: the file path
  '''
  filename = ''
  if interface_url:
    if interface_url.startswith('file:///'):
      filename = interface_url[7:]
    elif interface_url.startswith('package://') or interface_url.startswith('pkg://'):
      length = 6 if interface_url.startswith('pkg://') else 10
      pkg_name, _, pkg_path = interface_url[length:].partition('/')
      if pkg_path.startswith('//'):
        paths = roslib.packages.find_resource(pkg_name, pkg_path.strip('/'))
        if len(paths) > 0:
          # if more then one launch file is found, take the first one
          filename = paths[0]
      else:
        pkg_dir = roslib.packages.get_pkg_dir(pkg_name)
        filename = os.path.join(pkg_dir, pkg_path)
    else:
      filename = interface_url
    if filename == '.':
      filename = ''
    if filename and not os.path.exists(filename):
      raise ValueError('unsupported interface URL or interface file not found: ' + filename)
  return filename

def read_interface(interface_file):
  '''
  Reads the given file. You can use :mod:`master_discovery_fkie.common.resolve_url()`
  to resolve an URL to a file.
  
  :param interface_file: the file containing the interface.
  
  :type interface_file: str
  
  :raise ValueError: on error while read interface
  
  :return: directory with content of the given file
  '''
  data = {}
  with open(interface_file, 'r') as f: 
    iface = f.read() 
    # parse Interface file / YAML text
    # - lazy import 
    import yaml 
    try: 
      data = yaml.load(iface) 
      if data is None: 
        data = {} 
    except yaml.MarkedYAMLError, e: 
      if not interface_file:  
        raise ValueError("Error within YAML block:\n\t%s\n\nYAML is:\n%s"%(str(e), iface)) 
      else: 
        raise ValueError("file %s contains invalid YAML:\n%s"%(interface_file, str(e))) 
    except Exception, e: 
      if not interface_file: 
        raise ValueError("invalid YAML: %s\n\nYAML is:\n%s"%(str(e), iface)) 
      else: 
        raise ValueError("file %s contains invalid YAML:\n%s"%(interface_file, str(e))) 
  return data
  
def create_pattern(param, data, has_interface, default=[], mastername=''):
  '''
  Create and compile the regular expression for given parameter. The data is
  taken from `data`. If the data was read from the interface file, then you have
  to set the `has_interface` to True. If `has_interface` is False, the data will 
  be ignored and the parameter will be read from ROS parameter server.
  If resulting value is an empty list, `\\\\b` (http://docs.python.org/2/library/re.html)
  will be added to the pattern as `EMPTY_PATTERN`.
  
  :param param: parameter name
  
  :type param: str
    
  :param data: The dictionary, which can contain the parameter name and value. 
               The `data` will be ignored, if `has_interface` is `False`.
  
  :type data: dict
  
  :param has_interface: `True`, if valid data is available.
  
  :type has_interface: bool
  
  :param default: Default value will be added to the data
  
  :type default: list
  
  :return: the compiled regular expression
  
  :rtype: The result of `re.compile()`
  '''
  def_list = default
  if has_interface: # read the parameter from the sync interface data
    if data.has_key(param) and data[param]:
      for item in data[param]:
        if isinstance(item, dict):
          # this are mastername specific remapings
          if mastername and item.has_key(mastername):
            if isinstance(item[mastername], list):
              def_list[len(def_list):] = item[mastername]
            else:
              def_list.append(item[mastername])
        elif isinstance(item, list):
          def_list[len(def_list):] = item
        else:
          def_list.append(item)
  else: # reads the patterns from the ROS parameter server
    rp = rospy.get_param('~'+param, [])
    if isinstance(rp, list):
      def_list[len(def_list):] = rp
    else:
      def_list.append(rp)
    # reads the mastername specific parameters
    if mastername:
      rph = rospy.get_param('~'+roslib.names.ns_join(mastername, param), [])
      if isinstance(rp, list):
        def_list[len(def_list):] = rph
      else:
        def_list.append(rph)
  def_list = list(set(def_list))
  rospy.loginfo("%s: %s", param, str(def_list))
  def_list[:] = [''.join(['\A', n.strip().replace('*','.*'), '\Z']) for n in def_list]
  if def_list:
    return re.compile('|'.join(def_list), re.I)
  return EMPTY_PATTERN

def is_empty_pattern(re_object):
  '''
  Returns the value of `EMPTY_PATTERN`.
  '''
  return re_object == EMPTY_PATTERN
