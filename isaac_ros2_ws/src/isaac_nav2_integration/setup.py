from setuptools import find_packages, setup

package_name = 'isaac_nav2_integration'

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
    description='Nav2 navigation integration with NVIDIA Isaac for humanoid robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_nav2_controller = isaac_nav2_integration.nav2_controller:main',
            'isaac_path_planner = isaac_nav2_integration.path_planner:main',
            'isaac_behavior_manager = isaac_nav2_integration.behavior_manager:main',
        ],
    },
)