from setuptools import find_packages, setup

package_name = "perception_adapter"
setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sevnova_fsm",
    maintainer_email="dev@sevnova.local",
    description="External perception adapter node skeleton.",
    license="Proprietary",
    entry_points={"console_scripts": ["perception_adapter_node = perception_adapter.node:main"]},
)
