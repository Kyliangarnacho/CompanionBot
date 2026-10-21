from glob import glob
from setuptools import find_packages, setup

setup(
    name="companionbot_perception", version="0.9.1",
    packages=find_packages(include=["companionbot_perception", "companionbot_perception.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/companionbot_perception"]),
        ("share/companionbot_perception", ["package.xml", "README.md"]),
        ("share/companionbot_perception/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], extras_require={"test": ["pytest"]},
    zip_safe=False, maintainer="CompanionBot",
    maintainer_email="maintainer@companionbot.local",
    description="Camera, YOLO, ByteTrack, Master, ReID, depth and geometry", license="LicenseRef-CompanionBot",
    entry_points={"console_scripts": ["camera_tf = companionbot_perception.ros.camera_tf:main"]},
)

