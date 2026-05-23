from glob import glob

from setuptools import find_packages, setup

package_name = "fsm_test"
setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sevnova_fsm",
    maintainer_email="dev@sevnova.local",
    description="FSM tests and mock nodes.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "mock_perception_adapter_node = fsm_test.mocks.mock_perception_adapter_node:main",
            "mock_navigation_manager_node = fsm_test.mocks.mock_navigation_manager_node:main",
            "mock_pair_grasp_execution_node = fsm_test.mocks.mock_pair_grasp_execution_node:main",
            "mock_vacuum_io_node = fsm_test.mocks.mock_vacuum_io_node:main",
            "mock_chassis_status_publisher = fsm_test.mocks.mock_chassis_status_publisher:main",
            "mock_safety_button = fsm_test.mocks.mock_safety_button:main",
        ],
    },
)
