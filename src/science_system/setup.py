import os

from setuptools import find_packages, setup

package_name = "science_system"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include config folder
        (os.path.join("share", package_name, "config"), ["config/science_system_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jake Kemple / Trickfire",
    maintainer_email="jakek927@uw.edu",
    description="Science System Software Architecture",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Node entry points
            "science_hardware_node = science_system.science_hardware_node:main",
            "science_mission_node = science_system.science_mission_node:main",
        ],
    },
)
