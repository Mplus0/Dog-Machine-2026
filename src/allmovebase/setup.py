#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    py_modules=[
        "task_budget",
        "dog_arm_task_client",
    ],
    package_dir={"": "scripts"},
)

setup(**setup_args)
