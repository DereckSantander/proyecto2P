from setuptools import setup
import os
from glob import glob

package_name = 'go2_local_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dereck',
    maintainer_email='example@example.com',
    description='Local PID controller for following a global Path',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = go2_local_controller.pid_controller:main',
        ],
    },
)
