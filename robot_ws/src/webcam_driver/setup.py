from setuptools import setup

package_name = 'webcam_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='allentao',
    maintainer_email='allentao7@gmail.com',
    description='Dead simple driver for USB webcam',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = webcam_driver.driver:main'
        ],
    },
)
