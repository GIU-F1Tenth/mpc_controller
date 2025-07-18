"""Setup configuration for mpc_controller package."""

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mpc_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'casadi>=3.5.0',
        'numpy>=1.21.0',
        'pandas>=1.3.0',
        'transforms3d>=0.4.0',
        'matplotlib>=3.5.0',
        'scipy>=1.7.0',
    ],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='Model Predictive Controller for F1Tenth autonomous vehicle',
    license='MIT',
    tests_require=['pytest>=7.0.0', 'pytest-cov>=4.0.0'],
    entry_points={
        'console_scripts': [
            'mpc_node = mpc_controller.MPCtrlNode:main',
            'trajectory_optimizer = mpc_controller.optimize_trajectoryMPC:main',
        ],
    },
)
