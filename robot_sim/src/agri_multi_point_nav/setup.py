from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'agri_multi_point_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhang',
    maintainer_email='1721395658@qq.com',
    description='Multi-point navigation task management for an agricultural robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_point_nav = agri_multi_point_nav.multi_point_nav:main',
        ],
    },
)
