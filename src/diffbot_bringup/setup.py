from setuptools import find_packages, setup

package_name = 'diffbot_bringup'
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
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
	(os.path.join('share', package_name,'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bumpy-alpha',
    maintainer_email='bumpy-alpha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "simple_controller=diffbot_bringup.simple_controller:main",
            "simple_navigation=diffbot_bringup.simple_navigation:main",
            "oled_display=diffbot_bringup.oled_display_node:main"
        ],
    },
)
