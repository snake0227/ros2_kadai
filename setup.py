from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kadai'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/system_monitor_launch*')),
    ], 
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snake',
    maintainer_email='Github0227@gmail.com',
    description='a package for kadai',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'disk_monitor = kadai.disk_usage_monitor:main',
            'disk_sub = kadai.disk_info_sub:main',
],
    },
)
