from glob import glob
from setuptools import setup

setup(name="companionbot_ros", version="0.9.0", packages=["companionbot_ros"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/companionbot_ros"]),
                  ("share/companionbot_ros", ["package.xml"]),
                  ("share/companionbot_ros/launch", glob("launch/*.launch.py"))],
      install_requires=["setuptools"], extras_require={"test": ["pytest"]}, zip_safe=True,
      maintainer="CompanionBot", maintainer_email="maintainer@companionbot.local",
      description="Reuse existing CompanionBot runtimes through ROS 2",
      license="LicenseRef-CompanionBot",
      entry_points={"console_scripts": [
          "interaction = companionbot_ros.entry:main",
          "frozen_sim = companionbot_ros.frozen_sim:main",
          "camera_tf = companionbot_ros.camera_tf:main"]})
