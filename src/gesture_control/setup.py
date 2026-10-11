import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gesture_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={'gesture_control': ['models/*.task']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('gesture_control/models/*.task')),
        (os.path.join('lib', package_name, 'models'), glob('gesture_control/models/*.task')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sundar',
    maintainer_email='sundaramn353@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_node = gesture_control.gesture_node:main',
        ],
    },
)
