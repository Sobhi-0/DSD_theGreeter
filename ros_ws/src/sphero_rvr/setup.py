from setuptools import find_packages, setup

package_name = 'sphero_rvr'

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
    maintainer='Jonas Schmidt',
    maintainer_email='jst24008@student.mdu.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sphero_rvr_node = sphero_rvr.sphero_rvr_node:main'
        ],
    },
)
