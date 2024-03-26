from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'localizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Allen Tao',
    maintainer_email='allentao7@gmail.com',
    description='Simple 2D Localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localizer = localizer.localizer:main',
            'model_odom = localizer.model_odom:main',
            'tag_odom = localizer.tag_odom:main',
            'teleop_keyboard = localizer.teleop_keyboard:main'
        ],
    },
)
