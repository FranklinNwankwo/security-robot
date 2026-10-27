from glob import glob
from setuptools import setup

package_name = 'security_robot_alerts'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slim',
    maintainer_email='slim@todo.todo',
    description='Alerts backend and web dashboard for the security robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'alerts_server = security_robot_alerts.alerts_server:main',
        ],
    },
)
