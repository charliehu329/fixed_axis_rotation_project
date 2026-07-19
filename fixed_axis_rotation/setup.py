#!/usr/bin/env python3
"""
setup.py

功能：
    配置 fixed_axis_rotation ROS 2 Python 包的安装内容。

安装内容：
    1. fixed_axis_rotation Python 模块。
    2. package.xml。
    3. launch 启动文件。
    4. config 参数文件。
    5. 两个 ROS 2 Python 节点入口。
"""

import os
from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = "fixed_axis_rotation"


setup(
    name=package_name,
    version="0.0.1",

    packages=find_packages(
        exclude=[
            "test",
        ]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [
                os.path.join(
                    "resource",
                    package_name
                )
            ]
        ),
        (
            os.path.join(
                "share",
                package_name
            ),
            [
                "package.xml"
            ]
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch"
            ),
            glob(
                "launch/*.launch.py"
            )
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config"
            ),
            glob(
                "config/*.yaml"
            )
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="harry",
    maintainer_email="harry@example.com",

    description=(
        "Franka FR3 fixed-axis rotation "
        "controller and keyboard command node."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "fixed_axis_rotation_node = "
                "fixed_axis_rotation."
                "fixed_axis_rotation_node:main"
            ),
            (
                "fixed_axis_keyboard_node = "
                "fixed_axis_rotation."
                "fixed_axis_keyboard_node:main"
            ),
        ],
    },
)