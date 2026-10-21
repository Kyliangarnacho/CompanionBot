from glob import glob
from setuptools import find_packages, setup

setup(
    name="companionbot_behavior", version="0.9.1",
    packages=find_packages(include=["companionbot_behavior", "companionbot_behavior.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/companionbot_behavior"]),
        ("share/companionbot_behavior", ["package.xml", "README.md"]),
        ("share/companionbot_behavior/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], extras_require={"test": ["pytest"]},
    zip_safe=False, maintainer="CompanionBot",
    maintainer_email="maintainer@companionbot.local",
    description="Existing Agent, Catalog, Session and deterministic BehaviorSupervisor", license="LicenseRef-CompanionBot",
    entry_points={"console_scripts": ["interaction = companionbot_behavior.ros.entry:main"]},
)

