from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hexapod_control'  # Make sure this matches your package.xml

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install Launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # 2. Install Config files (YAML) - THIS IS THE NEW PART
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Control package for hexapod',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add your scripts here, e.g.:
            # 'hexapod_driver = hexapod_control.hexapod_driver:main',
        ],
    },
)