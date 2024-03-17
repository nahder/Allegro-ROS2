from setuptools import setup, find_packages

package_name = "gesture_classifier"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    package_data={
        "gesture_classifier.submodules.model": ["*.*", "*/*.*", "*/*/*.*"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="naderahmed",
    maintainer_email="ndr.ahmed1@gmail.com",
    description="A ROS package for gesture classification using TensorFlow and MediaPipe",
    license="Specify your license",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gesture_classifier = gesture_classifier.gesture_classifier:main",
        ],
    },
)
