from setuptools import setup, find_packages

package_name = 'puma_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junryeol',
    maintainer_email='junryeol@todo.todo',
    description='PUMA 3DOF ROS2 + WebSocket Integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puma_controller = puma_robot.puma_controller:main',
            'ros2_web_bridge = puma_robot.ros2_web_bridge:main',
            'joint_state_logger = puma_robot.joint_state_logger:main',
            'test = puma_robot.test:main'
        ],
    },
)
