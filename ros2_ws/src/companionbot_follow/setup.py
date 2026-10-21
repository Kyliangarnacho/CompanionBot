from glob import glob
from setuptools import find_packages, setup

setup(
    name="companionbot_follow", version="0.9.1",
    packages=find_packages(include=["companionbot_follow", "companionbot_follow.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/companionbot_follow"]),
        ("share/companionbot_follow", ["package.xml", "README.md"]),
        ("share/companionbot_follow/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], extras_require={"test": ["pytest"]},
    zip_safe=False, maintainer="CompanionBot",
    maintainer_email="maintainer@companionbot.local",
    description="Pi-side following estimation, governor and reference planning", license="LicenseRef-CompanionBot",
    entry_points={"console_scripts": []},
)

