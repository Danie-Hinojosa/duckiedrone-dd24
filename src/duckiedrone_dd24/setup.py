from glob import glob
from setuptools import setup

package_name = 'duckiedrone_dd24'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/sim/airframes', glob('sim/airframes/*')),
        ('share/' + package_name + '/sim/models/dd24', ['sim/models/dd24/model.config',
                                                        'sim/models/dd24/model.sdf']),
        ('share/' + package_name + '/sim/models/dd24/meshes', glob('sim/models/dd24/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Eduardo Hinojosa Alvarado',
    maintainer_email='A00838156@tec.mx',
    description='Modelo, visualización y control de altura del Duckiedrone DD24-B.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'hover_test = duckiedrone_dd24.hover_test:main',
            'motor_monitor = duckiedrone_dd24.motor_monitor:main',
            'drone_state_bridge = duckiedrone_dd24.drone_state_bridge:main',
            'scan_to_range = duckiedrone_dd24.scan_to_range:main',
        ],
    },
)
