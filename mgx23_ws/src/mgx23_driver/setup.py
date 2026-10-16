from setuptools import find_packages, setup

package_name = 'mgx23_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhang',
    maintainer_email='1721395658@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    "console_scripts": [
        "mgx23_node = mgx23_driver.mgx23_node:main",
        "mgx23_client = mgx23_driver.mgx23_client:main"
    ],
},
)
