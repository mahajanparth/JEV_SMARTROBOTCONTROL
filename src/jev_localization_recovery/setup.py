from glob import glob
from setuptools import find_packages, setup

setup(
    name='jev_localization_recovery', version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'jev_localization_recovery': ['dashboard.html', 'safety.html']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/jev_localization_recovery']),
        ('share/jev_localization_recovery', ['package.xml']),
        ('share/jev_localization_recovery/launch', glob('launch/*.launch.py')),
        ('share/jev_localization_recovery/config', glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.xml')),
        ('share/jev_localization_recovery/maps', glob('maps/*')),
        ('share/jev_localization_recovery/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Jev Demo', maintainer_email='demo@example.com',
    description='Simple localization health monitoring and bounded recovery.',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mission_node = jev_localization_recovery.mission_node:main',
        'mission_bridge = jev_localization_recovery.mission_bridge:main',
        'recovery_node = jev_localization_recovery.recovery_node:main',
        'inject_delocalization = jev_localization_recovery.inject_delocalization:main',
        'gazebo_support = jev_localization_recovery.gazebo_support:main',
    ]},
)
