from glob import glob
from setuptools import setup

package_name = 'security_robot_security'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slim',
    maintainer_email='slim@todo.todo',
    description='Security logic for the security robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'zone_monitor = security_robot_security.zone_monitor:main',
            'security_engine = security_robot_security.security_engine:main',
        ],
    },
)
