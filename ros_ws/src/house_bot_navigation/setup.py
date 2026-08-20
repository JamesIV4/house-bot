from glob import glob
import os

from setuptools import find_packages, setup


package_name = "house_bot_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="James",
    maintainer_email="james@housebot.local",
    description="House Bot navigation orchestration and UI adapters",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_driver = house_bot_navigation.base_driver:main",
            "named_goal_manager = house_bot_navigation.named_goal_manager:main",
            "mock_initial_pose = house_bot_navigation.mock_initial_pose:main",
            "navigation_smoke_test = house_bot_navigation.smoke_test:main",
        ],
    },
)
