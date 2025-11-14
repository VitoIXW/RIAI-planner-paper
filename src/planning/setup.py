from setuptools import find_packages, setup

package_name = 'planning'

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Domínguez Montero',
    maintainer_email='mandominguez97@gmail.com',
    description="planning",
    license='private',
    tests_require=['pytest'],
)
