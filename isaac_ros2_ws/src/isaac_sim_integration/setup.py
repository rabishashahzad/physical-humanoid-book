from setuptools import find_packages, setup

package_name = 'isaac_sim_integration'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='ROS 2 integration with NVIDIA Isaac Sim for robotics simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_sim_bridge = isaac_sim_integration.isaac_sim_bridge:main',
            'sensor_publisher_node = isaac_sim_integration.sensor_publisher_node:main',
        ],
    },
)