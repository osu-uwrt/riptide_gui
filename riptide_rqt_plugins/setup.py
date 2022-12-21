from setuptools import setup
from glob import glob
import os

package_name = 'riptide_rqt_plugins2'
setup(
    name=package_name,
    version='1.0.1',
    package_dir={'': 'src'}, 
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name), ['plugin.xml']),
        (os.path.join('share', package_name), ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Robert Pafford',
    maintainer='OSU UWRT',
    maintainer_email='osu.uwrt@gmail.com',
    keywords=['ROS', 'riptide'],
    description=(
        'RQT Plugins for Riptide specific features'
    ),
    license='BSD'
)