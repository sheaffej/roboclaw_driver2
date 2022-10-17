import os
from glob import glob
from setuptools import setup

package_name = 'roboclaw_driver2'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='sheaffej@gmail.com',
    description='The ROS2 roboclaw_driver package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboclaw_node = roboclaw_driver2.roboclaw_node:main'
        ],
    },
)
