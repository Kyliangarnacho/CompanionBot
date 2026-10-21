from glob import glob
from setuptools import find_packages, setup

setup(
    name="companionbot_control", version="0.9.1",
    packages=find_packages(include=["companionbot_control", "companionbot_control.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/companionbot_control"]),
        ("share/companionbot_control", ["package.xml", "README.md"]),
        ("share/companionbot_control/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], extras_require={"test": ["pytest"]},
    zip_safe=False, maintainer="CompanionBot",
    maintainer_email="maintainer@companionbot.local",
    description="Frozen MCU-side control, estimation and reference safety libraries", license="LicenseRef-CompanionBot",
    entry_points={"console_scripts": []},
)

