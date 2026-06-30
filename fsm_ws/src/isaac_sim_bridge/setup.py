from setuptools import find_packages, setup

package_name = "isaac_sim_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sevnova_fsm",
    maintainer_email="dev@sevnova.local",
    description="Isaac Sim ground-truth and chassis bridge nodes for FSM full-flow simulation.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "isaac_ground_truth_perception_node = isaac_sim_bridge.ground_truth_perception_node:main",
            "isaac_chassis_bridge_node = isaac_sim_bridge.chassis_bridge_node:main",
        ]
    },
)
