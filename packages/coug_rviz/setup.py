from setuptools import find_packages, setup
import os
from glob import glob

package_name = "coug_rviz"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nelson Durrant",
    maintainer_email="snelsondurrant@gmail.com",
    description="RViz GUI for the CougUV",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "odom_to_path = coug_rviz.odom_to_path_node:main",
        ],
    },
)
