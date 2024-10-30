import os
from glob import glob

from setuptools import setup

package_name = "autonomous_nav_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="your_email@example.com",
    description="Autonomous navigation nodes",
    license="Your License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigation_node = autonomous_nav_pkg.navigation_node:main",
            "sensor_processing_node = autonomous_nav_pkg.sensor_processing_node:main",
            "control_node = autonomous_nav_pkg.control_node:main",
            "localization_node = autonomous_nav_pkg.localization_node:main",
            "decision_making_node = autonomous_nav_pkg.decision_making_node:main",
        ],
    },
)
