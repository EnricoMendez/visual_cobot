from setuptools import find_packages, setup
from glob import glob

package_name = 'visual_cobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.task')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enrico',
    maintainer_email='enrico.mendez@outlook.es',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_recognition = visual_cobot.gesture_recognition:main',
            'visual_control = visual_cobot.visual_control:main',
            'visual_control_sim = visual_cobot.visual_control_sim:main',
        ],
    },
)
