#! /usr/bin/env python
# from setuptools import setup
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dual_sawyer_controller'],
    # scripts=['scripts'],
    package_dir={'': 'src'}
)

setup(**d)
