from setuptools import find_packages, setup

package_name = "autonomous_arm"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="trickfire",
    maintainer_email="alexsing@uw.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "typing_coordinator = arm.autonomous_arm.typing_coordinator_node:main",
            "keyboard_locator = arm.autonomous_arm.keyboard_locator_node:main",
            "keyboard_pose = arm.autonomous_arm.keyboard_pose_node:main",
            "arm_control = arm.autonomous_arm.arm_control_node:main",
        ],
    },
)
