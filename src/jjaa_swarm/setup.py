from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jjaa_swarm'

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ('share/' + package_name + '/configuration', glob('configuration/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Domínguez Montero',
    maintainer_email='mandominguez97@gmail.com',
    description="jjaa_swarm",
    license='private',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jjaa_swarm = jjaa_swarm.jjaa_swarm:main', 
        ],
    },
)
