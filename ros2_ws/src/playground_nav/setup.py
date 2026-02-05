from setuptools import setup
import os
from glob import glob

package_name = "playground_nav"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "launch", "config"),
            glob("launch/config/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Playground Maintainer",
    maintainer_email="playground@example.com",
    description="Navigation package for O3DE ROS2 Playground",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
