import os
from setuptools import find_packages, setup
from glob import glob
package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/camera_pkg']),
    ('share/camera_pkg', ['package.xml']),
    (os.path.join('share', 'camera_pkg', 'launch'), glob('launch/*.py')),
    (os.path.join('share', 'camera_pkg', 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isaacp',
    maintainer_email='isaac.peterson@usu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'perception = camera_pkg.perception:main',
            'control = camera_pkg.control:main',
        ],
    },
)
