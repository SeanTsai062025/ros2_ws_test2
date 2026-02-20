from setuptools import setup

package_name = 'dexter_control_panel'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SeanTsai',
    maintainer_email='seantsai06@gmail.com',
    description='Interactive slider GUI for controlling the Dexter robot arm',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'control_panel = dexter_control_panel.control_panel:main',
        ],
    },
)
