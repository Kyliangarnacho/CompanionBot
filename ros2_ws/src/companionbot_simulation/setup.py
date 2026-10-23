from glob import glob
from setuptools import find_packages, setup

setup(
    name="companionbot_simulation", version="0.9.1",
    packages=find_packages(include=["companionbot_simulation", "companionbot_simulation.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/companionbot_simulation"]),
        ("share/companionbot_simulation", ["package.xml", "README.md"]),
        ("share/companionbot_simulation/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], extras_require={"test": ["pytest"]},
    zip_safe=False, maintainer="CompanionBot",
    maintainer_email="maintainer@companionbot.local",
    description="MuJoCo plant, synthetic sensors and frozen scenario composition", license="LicenseRef-CompanionBot",
    entry_points={"console_scripts": ["frozen_sim = companionbot_simulation.ros.frozen_sim:main",
                                     "spatial_sim = companionbot_simulation.ros.spatial_sim:main"]},
)

