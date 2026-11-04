from setuptools import find_packages, setup

package_name = "fish_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fish_ws maintainer",
    maintainer_email="user@example.com",
    description="Single-key experiment coordination and rosbag2 auditing tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "keyboard_console = fish_tools.keyboard_console:main",
            "experiment_manager = fish_tools.experiment_manager:main",
            "bag_audit = fish_tools.bag_audit:main",
        ],
    },
)
