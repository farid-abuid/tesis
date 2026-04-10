from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'exo_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.config')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'matplotlib',
        'pyserial',
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
    scripts=[
        os.path.join(os.path.dirname(__file__), 'scripts', 'exo_data_logger'),
        os.path.join(os.path.dirname(__file__), 'scripts', 'exo_plot_run'),
        os.path.join(os.path.dirname(__file__), 'scripts', 'exo_dc_motor_id'),
    ],
)
