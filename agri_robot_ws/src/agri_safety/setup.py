from glob import glob
from setuptools import find_packages, setup

package_name = 'agri_safety'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zhang', maintainer_email='1721395658@qq.com',
    description='RTK/INS agricultural navigation', license='Apache-2.0',
    entry_points={'console_scripts': ["safety = agri_safety.node:main"]},
)
