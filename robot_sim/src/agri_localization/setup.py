from setuptools import find_packages, setup

package_name = 'agri_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=[
        'setuptools',
        'pyproj',
    ],
    zip_safe=True,
    maintainer='zhang',
    maintainer_email='1721395658@qq.com',
    description='Agricultural robot localization and coordinate conversion',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coordinate_converter = agri_localization.coordinate_converter:main',
            'gps_map_tf_publisher = agri_localization.gps_map_tf_publisher:main',
        ],
    },
)