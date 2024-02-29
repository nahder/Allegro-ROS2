from setuptools import find_packages, setup

package_name = "allegro_game"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "launch/start_game.launch.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="naderahmed",
    maintainer_email="ndr.ahmed1@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "allegro_game = allegro_game.allegro_game:main",
            "delay = allegro_game.delay:main",
        ],
    },
)
