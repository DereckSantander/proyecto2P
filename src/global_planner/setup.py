from setuptools import setup
import os
from glob import glob

package_name = 'global_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dereck',
    maintainer_email='example@example.com',
    description='Global path planner using Dijkstra algorithm',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dijkstra_planner = global_planner.dijkstra_planner:main',
        ],
    },
)
