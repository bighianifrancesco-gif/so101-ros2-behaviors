from setuptools import setup

package_name = 'so101_hardware_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='francesco',
    maintainer_email='bighiani.francesco@gmail.com',
    description='Hardware interface nodes for SO-101 and IMU',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_serial_node = so101_hardware_py.imu_serial_node:main',
            'arm_interface_node = so101_hardware_py.arm_interface_node:main',
            'behaviour_controller_node = so101_control_py.behaviour_controller_node:main',
        ],
    },
)
