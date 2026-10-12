from glob import glob

from setuptools import find_packages, setup

package_name = 'bld120_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Itai Groisman',
    maintainer_email='itaigro@gmail.com',
    description='BLD-120MM simulation with firmware-realistic encoder odometry',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odometry = bld120_sim.encoder_odometry_node:main',
            'virtual_firmware = bld120_sim.virtual_firmware_node:main',
            'square_driver = bld120_sim.square_driver_node:main',
            'pose_recorder = bld120_sim.pose_recorder_node:main',
            'analyze_square = bld120_sim.analyze_square:main',
            'step_test = bld120_sim.step_test_node:main',
            'analyze_steps = bld120_sim.analyze_steps:main',
        ],
    },
)
