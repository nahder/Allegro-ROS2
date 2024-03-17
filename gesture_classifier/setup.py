from setuptools import setup, find_packages

package_name = "gesture_classifier"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    package_data={
        # Attempt to include everything in the model directory
        # This pattern assumes all subdirectories under model are Python packages
        "gesture_classifier.submodules.model": ["*.*", "*/*.*", "*/*/*.*"],
    },
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
