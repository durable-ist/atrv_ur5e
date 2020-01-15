#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['localization_switch', 'localization_switch_ros'],
 package_dir={'localization_switch': 'common/src/localization_switch', 'localization_switch_ros': 'ros/src/localization_switch_ros'}
)

setup(**d)
