from glob import glob
import os

from setuptools import find_packages, setup

PACKAGE_NAME = "yahboom_rosmaster_hw_bridge"

setup(
    name=PACKAGE_NAME,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            [os.path.join("resource", PACKAGE_NAME)]),
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Milstein",
    maintainer_email="dnmilstein@miuandes.cl",
    description="Pragmatic ROS 2 hardware bridge for the Yahboom ROSMaster X3 Plus.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"arm_action_server = {PACKAGE_NAME}.arm_action_server:main",
            f"gripper_action_server = {PACKAGE_NAME}.gripper_action_server:main",
            f"joint_state_publisher = {PACKAGE_NAME}.joint_state_publisher:main",
            f"base_driver = {PACKAGE_NAME}.base_driver:main",
        ],
    },
)
