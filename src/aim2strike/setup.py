# from setuptools import find_packages, setup

# package_name = 'aim2strike'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='robotech',
#     maintainer_email='robotech@todo.todo',
#     description='TODO: Package description',
#     license='Apache-2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )
from setuptools import setup

package_name = 'aim2strike'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/camera_node.yaml']),
        ('share/' + package_name + '/config', ['config/aim_node.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 Camera Node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = aim2strike.camera_node:main',
            'detect_node = aim2strike.detect_node:main',
            'aim_node = aim2strike.aim_node:main',
            'uart_servo.py = aim2strike.uart_servo:main',
        ],
    },
)