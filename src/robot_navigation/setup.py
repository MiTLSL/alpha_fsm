from glob import glob

from setuptools import find_packages, setup

package_name = "robot_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sevnova",
    maintainer_email="dev@sevnova.local",
    description="Navigation validation nodes for Alfa Isaac Sim integration.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "cmd_vel_to_alfa_node = robot_navigation.cmd_vel_to_alfa_node:main",
            "nav_goal_sender = robot_navigation.nav_goal_sender:main",
        ],
    },
)
