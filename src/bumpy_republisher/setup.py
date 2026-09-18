from setuptools import find_packages, setup

package_name = 'bumpy_republisher'
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
        (os.path.join("share/", package_name, "urdf"), glob("urdf/*")),
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
            "republish_cmd_vel=bumpy_republisher.republish_cmd_vel:main",
            "republish_imu_tf=bumpy_republisher.republish_imu_tf:main",
            "republish_odom_tf=bumpy_republisher.republish_odom_tf:main",
            "republish_scan=bumpy_republisher.republish_scan:main",
        ],
    },
)
