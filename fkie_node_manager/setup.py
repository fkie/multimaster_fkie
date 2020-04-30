#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   ##  don't do this unless you want a globally visible script
   scripts=['nodes/node_manager'], 
   packages=['fkie_node_manager', 'fkie_node_manager.editor', 'fkie_node_manager.nmd_client', 'fkie_node_manager.logscreen'],
   package_dir={'': 'src'}
)

setup(**d)