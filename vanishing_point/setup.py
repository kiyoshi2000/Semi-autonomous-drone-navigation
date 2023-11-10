from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vanishing_point'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
      data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.rviz"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.perspective"),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.model"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st5dronelab',
    maintainer_email='st5dronelab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vp_node = vanishing_point.vp_node:main",
            "republish = vanishing_point.republish:main"
        ],
    },
)
