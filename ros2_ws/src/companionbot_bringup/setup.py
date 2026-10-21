from glob import glob
from setuptools import find_packages, setup

setup(
    name="companionbot_bringup", version="0.9.1",
    packages=find_packages(include=["companionbot_bringup", "companionbot_bringup.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/companionbot_bringup"]),
        ("share/companionbot_bringup", ["package.xml", "README.md"]),
        ("share/companionbot_bringup/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], extras_require={"test": ["pytest"]},
    zip_safe=False, maintainer="CompanionBot",
    maintainer_email="maintainer@companionbot.local",
    description="The sole formal launch and explicitly diagnostic simulation launch", license="LicenseRef-CompanionBot",
    entry_points={"console_scripts": []},
)

