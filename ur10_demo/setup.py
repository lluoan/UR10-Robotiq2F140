## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['<package_name>'],
    # scripts=['/scripts'],s
    package_dir={'': 'include'},
)
setup(**setup_args)

#!/usr/bin/env python
# -*- coding: utf-8 -*-


# from distutils.core import setup
# from catkin_pkg.python_setup import generate_distutils_setup


# setup_args = generate_distutils_setup(
#     packages=['<\'],

#     package_dir={'': 'src'}
# )

# setup(**setup_args)