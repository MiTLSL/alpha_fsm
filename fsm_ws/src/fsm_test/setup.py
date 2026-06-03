from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "fsm_test"


def data_files_under(directory: str):
    root = Path(directory)
    if not root.exists():
        return []
    groups = []

    def should_install(path: Path) -> bool:
        rel_parts = path.relative_to(root).parts
        if rel_parts and rel_parts[0] == "supplied":
            return False
        return ":Zone.Identifier" not in path.name

    for parent in sorted(
        {path.parent for path in root.rglob("*") if path.is_file() and should_install(path)}
    ):
        files = [
            str(path)
            for path in sorted(parent.iterdir())
            if path.is_file() and should_install(path)
        ]
        if files:
            groups.append((f"share/{package_name}/{parent}", files))
    return groups


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ]
    + data_files_under("l3_sim_03_assets"),
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
            "sim_world_node = fsm_test.sim.sim_world_node:main",
            "fake_nav2_base_node = fsm_test.sim.fake_nav2_base_node:main",
        ],
    },
)
