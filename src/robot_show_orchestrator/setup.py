from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'robot_show_orchestrator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='sean',
    maintainer_email='seantsai06@gmail.com',
    description='Correlated YAML workflow runtime for the Dexter robot show',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'show_orchestrator = '
            'robot_show_orchestrator.orchestrator_node:main',
        ],
    },
)
