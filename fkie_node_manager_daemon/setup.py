import os

package_name = 'fkie_node_manager_daemon'

if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == '1':
   from distutils.core import setup
   from catkin_pkg.python_setup import generate_distutils_setup

   d = generate_distutils_setup(
      ##  don't do this unless you want a globally visible script
      scripts=['nodes/node_manager_daemon'],
      packages=[package_name, '%s.monitor' % package_name],
      package_dir={'': 'src'}
   )

   setup(**d)
   exit(0)

### ROS2 ###

from setuptools import setup, Command
from setuptools.command.build_py import build_py
from datetime import date
from pkg_resources import resource_filename
import subprocess
import xml.etree.ElementTree as ET


resource_files = [
    'resources/description_example.launch.xml',
    'resources/include_dummy.launch.xml',
    'resources/included1.launch.xml',
    'resources/included2.launch.xml',
]


version = "0.0.0"


def get_version():
    if os.path.isdir("../.git"):
        try:
            ps = subprocess.Popen(['git', 'describe', '--tags', '--dirty', '--always', '--abbrev=8'], stdout=subprocess.PIPE)
            output = ps.stdout.read()
            vers = output.decode('utf-8').strip()
            parts = vers.split('.', 3)
            if len(parts) < 3:
                raise Exception('no version tag found in git')
            ps.wait()
            ps = subprocess.Popen(['git', 'show', '-s', '--format=%ci'], stdout=subprocess.PIPE)
            output = ps.stdout.read().split()
            if output:
                date_str = output[0].decode('utf-8')
            else:
                date_str = date.today().isoformat()
            ps.wait()
            return vers, date_str
        except Exception as _err:
            pass
            # print("git version detection error: %s" % err)
    tree = ET.parse('package.xml')
    root = tree.getroot()
    for vers in root.findall('version'):
        return vers.text, date.today().isoformat()
    return "0.0.0", date.today().isoformat()


def strip_dirty_vers(vers):
    parts = vers.split('-', 2)
    return parts[0]

class BuildPyCommand(build_py):
    def run(self):
        # honor the --dry-run flag
        if not self.dry_run:
            # read version from package.xml
            vers, version_date = get_version()
            if vers != '0.0.0':
                lib_dir = '%s/%s' % (self.build_lib, package_name)
                if not os.path.exists(lib_dir):
                    os.makedirs(lib_dir)
                with open('%s/pkg_version.py' % lib_dir, 'w+') as vf:
                    vf.write("version = '%s'\n" % vers)
                    vf.write("date = '%s'\n" % version_date)
                    # TODO: add version and date from git
        # run base class code
        super(BuildPyCommand, self).run()

setup(
    name=package_name,
    version=strip_dirty_vers(get_version()[0]),
    packages=[package_name, package_name + '.monitor', package_name + '.tests'],
    cmdclass={'build_py': BuildPyCommand},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/autostart.launch.xml']),
        ('share/' + package_name + '/resources', resource_files),
    ],
    install_requires=['setuptools', 'ruamel.yaml', 'launch-xml'],
    zip_safe=True,
    maintainer='Alexander Tiderko',
    maintainer_email='Alexander.Tiderko@fkie.fraunhofer.de',
    description='A daemon node to manage ROS launch files and launch nodes from loaded files.',
    license='Apache License, Version 2.0',
    url='https://github.com/fkie/ros_node_manager',
    tests_require=['pytest'],
    test_suite="tests",
    entry_points={
        'console_scripts': [
           'node_manager_daemon ='
           ' fkie_node_manager_daemon:main',
        ],
    },
)