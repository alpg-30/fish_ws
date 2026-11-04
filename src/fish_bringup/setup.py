from glob import glob
from setuptools import find_packages, setup

package_name = "fish_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fish_ws maintainer",
    maintainer_email="user@example.com",
    description="Mock-first launch and configuration for the fish experiment.",
    license="Apache-2.0",
)
