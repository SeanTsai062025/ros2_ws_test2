from setuptools import find_packages, setup

package_name = 'esp32_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='sean',
    maintainer_email='seantsai06@gmail.com',
    description='ROS2 UART bridge node for ESP32 motor controller communication',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uart_bridge_node = esp32_motor_controller.uart_bridge_node:main',
        ],
    },
)
