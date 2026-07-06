from setuptools import find_packages, setup

package_name = "isaac_visualization_bridge"

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
    description="Validation status and marker publisher for Isaac integration.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "validation_status_node = isaac_visualization_bridge.validation_status_node:main",
        ],
    },
)
