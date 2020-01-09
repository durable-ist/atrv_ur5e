#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['maps', 'maps_ros'],
 package_dir={'maps': 'common/src/maps', 'maps_ros': 'ros/src/maps_ros'}
)

setup(**d)
