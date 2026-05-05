from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'exo_bringup'

# Install YAML files from every config/<mode>/ subdirectory.
_config_data_files = [
    (os.path.join('share', package_name, 'config'),
     glob('config/*.yaml') + glob('config/*.config')),
]
for _mode_dir in sorted(glob('config/*/')):
    _mode = os.path.basename(_mode_dir.rstrip('/'))
    _yamls = glob(f'config/{_mode}/*.yaml')
    if _yamls:
        _config_data_files.append(
            (os.path.join('share', package_name, 'config', _mode), _yamls)
        )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['launch_common'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        *_config_data_files,
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='farid',
    maintainer_email='farid.abuid@utec.edu.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
