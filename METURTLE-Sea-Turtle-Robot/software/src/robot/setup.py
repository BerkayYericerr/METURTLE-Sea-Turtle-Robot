from setuptools import find_packages, setup

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],  # Added dependencies
    zip_safe=True,
    maintainer='efe',
    maintainer_email='efe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = robot.sender_node:main',
            'sub = robot.receiver_node:main',
            'robot_comm = robot.sender:main',
            'gps_old = robot.gps_test:main',
            'gps = robot.gps_arduino:main',
            'autopilot_old = robot.autopilot:main',
            'autopilot = robot.autopilot2:main',
            'esptest = robot.esptest:main',
            'detect = robot.detect:main',
            'compass = robot.compass_arduino:main',
        ],
    },
)
