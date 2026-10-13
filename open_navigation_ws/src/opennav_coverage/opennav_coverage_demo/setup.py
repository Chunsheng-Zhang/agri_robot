from glob import glob
import os

from setuptools import setup


package_name = 'opennav_coverage_demo'

# 辅助函数：只获取文件，排除目录（如 __pycache__）
def get_files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), get_files('launch/*')),
        (os.path.join('share', package_name), get_files('world/*')),
        (os.path.join('share', package_name), get_files('params/*')),
        (os.path.join('share', package_name), get_files('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stevenmacenski@gmail.com',
    description='An importable library for writing mobile robot applications in python3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'demo_coverage = opennav_coverage_demo.demo_coverage:main',
                'demo_row_coverage = opennav_coverage_demo.demo_row_coverage:main',
                'demo_coverage_via_headland = '
                'opennav_coverage_demo.demo_coverage_via_headland:main',
        ],
    },
)
