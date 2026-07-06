from setuptools import find_packages, setup

package_name = "robot_state_machine"

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
    maintainer="sevnova",
    maintainer_email="dev@sevnova.local",
    description="State-machine summary nodes for Alfa validation.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "state_summary_node = robot_state_machine.state_summary_node:main",
            "alfa_task_state_machine_node = robot_state_machine.alfa_task_state_machine_node:main",
            "alfa_truth_demo_state_machine_node = robot_state_machine.alfa_truth_demo_state_machine_node:main",
        ],
    },
)
