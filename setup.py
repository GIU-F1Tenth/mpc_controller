from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpc_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'casadi'],
    zip_safe=True,
    maintainer='MOhammed_Azab',
    maintainer_email='mo7ammed3zab@outlook.com',
    description='Model Predictive Control (MPC) for autonomous systems',
    license='MIT',
    tests_require=['pytest', 'pytest-cov'],
    entry_points={
        'console_scripts': [
            'MPC_Node = mpc_controller.MPCtrlNode:main'
        ],
    },
)
