import os
from glob import glob
from setuptools import setup

package_name = 'jetauto_rviz'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='opyntorr',
    maintainer_email='a00344869@tec.mx',
    description='RViz para monitorear el bridge JetAuto desde la laptop.',
    license='MIT',
    entry_points={'console_scripts': [
        'guardar_mapa = jetauto_rviz.guardar_mapa:main',
    ]},
)
