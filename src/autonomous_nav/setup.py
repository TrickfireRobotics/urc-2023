import os
from glob import glob

from setuptools import setup

package_name = "autonomous_nav"

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
    maintainer="Trickfire / Jake Kemple",
    maintainer_email="jakek927@uw.edu",
    description="Autonomous Navigation Nodes",
    license="Your License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigation_node = autonomous_nav.navigation_node:main",
            "sensor_processing_node = autonomous_nav.sensor_processing_node:main",
            "gps_anchor_node = autonomous_nav.gps_anchor_node:main",
            "decision_making_node = autonomous_nav.decision_making_node:main",
        ],
    },
)
