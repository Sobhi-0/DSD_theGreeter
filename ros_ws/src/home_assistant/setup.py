from setuptools import find_packages, setup

package_name = 'home_assistant'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='maheenghani',
    maintainer_email='maheenghani@todo.todo',
    description='Home Assistant ROS 2 Interface Node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'home_assistant_node = home_assistant.home_assistant_node:main'
        ],
    },
)
