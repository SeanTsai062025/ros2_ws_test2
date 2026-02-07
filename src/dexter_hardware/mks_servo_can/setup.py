"""Setup file for mks-servo-can library"""

from setuptools import setup, find_packages

setup(
    name='mks-servo-can',
    version='0.2.2',
    description='MKS Servo Motor CAN Bus Control Library',
    author='Dexter Robot Team',
    packages=find_packages(),
    install_requires=[
        'python-can>=3.0.0',
    ],
    python_requires='>=3.8',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
)
