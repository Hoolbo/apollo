import os
from glob import glob
from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hoolbo',
    maintainer_email='user@example.com',
    description='LTV-MPC controller for articulated tracked vehicle',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpc_node = controller.mpc_node:main',
        ],
    },
)
