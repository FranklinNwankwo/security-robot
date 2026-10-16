from glob import glob
from setuptools import setup

package_name = 'security_robot_webots'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
    ('share/' + package_name + '/resource', glob('resource/*.urdf')),
    ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slim',
    maintainer_email='slim@todo.todo',
    description='Webots simulation and driver for the security robot',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
