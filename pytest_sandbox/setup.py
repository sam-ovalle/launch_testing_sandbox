from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pytest_sandbox'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Include test files
        ('share/' + package_name + '/test', glob('test/test_*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Example chatter publisher with launch testing',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'chatter_publisher = pytest_sandbox.chatter_publisher:main',
        ],
    },
)