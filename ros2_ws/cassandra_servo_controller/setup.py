from glob import glob

from setuptools import find_packages, setup


package_name = "cassandra_servo_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cassandra maintainers",
    maintainer_email="maintainer@example.com",
    description="Lifecycle ROS 2 driver for Cassandra's LX-16A servos",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_controller = "
            "cassandra_servo_controller.servo_controller:main",
        ],
    },
)
