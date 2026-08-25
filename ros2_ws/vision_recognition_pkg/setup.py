from setuptools import setup

package_name = "vision_recognition_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name,
        package_name + ".face_detection",
        package_name + ".object_detection",
        package_name + ".tracking",
        package_name + ".utils",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vision_launch.py"]),
        (
            "share/" + package_name + "/models",
            ["models/haarcascade_frontalface_default.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@email.com",
    description="Vision recognition package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "face_detector = vision_recognition_pkg.face_detection.face_detector:main",
            "object_detector = vision_recognition_pkg.object_detection.object_detector:"
            "main",
        ],
    },
)
