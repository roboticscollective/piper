from setuptools import find_packages, setup

package_name = 'quest_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mk',
    maintainer_email='mrutyunjay.kalyani1999@gmail.com',
    description='Meta Quest VR controller teleoperation bridge for the Piper arm',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'quest_driver = quest_bridge.quest_driver:main',
            'vr_servo_bridge = quest_bridge.vr_servo_bridge:main',
            'quest_piper_teleop = quest_bridge.quest_piper_teleop:main',
        ],
    },
)
