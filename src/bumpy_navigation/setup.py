from setuptools import find_packages, setup

package_name = 'bumpy_navigation'
import os
from glob import glob
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share/", package_name, "launch"), glob("launch/*")),
        (os.path.join("share/", package_name, "config"), glob("config/*")),
        (os.path.join("share/", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share/", package_name, "behavior_trees"), glob("behavior_trees/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adem-vishal',
    maintainer_email='vishaladem101@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interactive_waypoints = bumpy_navigation.interactive_waypoints:main',
            'cmd_vel_republisher = bumpy_navigation.cmd_vel_republisher:main',
        ],
    },
)
