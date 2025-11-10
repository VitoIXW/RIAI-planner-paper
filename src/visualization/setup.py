from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'visualization'

def package_files(directory):
    paths = []
    for path in glob(os.path.join(directory, '**'), recursive=True):
        if os.path.isfile(path):  # Solo ficheros, no carpetas
            paths.append(path)
    return paths

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Copiar todo el contenido de assets recursivamente
        (os.path.join('share', package_name, 'assets'),
            package_files('assets')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Domínguez Montero',
    maintainer_email='mandominguez97@gmail.com',
    description="visualization",
    license='private',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualization = visualization.visualization:main', 
        ],
    },
)
