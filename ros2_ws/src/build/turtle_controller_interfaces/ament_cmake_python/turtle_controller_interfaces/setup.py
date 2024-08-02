from setuptools import find_packages
from setuptools import setup

setup(
    name='turtle_controller_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('turtle_controller_interfaces', 'turtle_controller_interfaces.*')),
)
