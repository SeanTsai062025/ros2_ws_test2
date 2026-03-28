from setuptools import find_packages, setup

package_name = 'motor_control_state_machine'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'transitions'],
    zip_safe=True,
    maintainer='sean',
    maintainer_email='seantsai06@gmail.com',
    description='ROS 2 motor control state machine with transitions library',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control_state_machine = motor_control_state_machine.motor_control_state_machine:main',
        ],
    },
)
