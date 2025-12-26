from setuptools import find_packages, setup

package_name = 'isaac_vslam_demos'

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
    description='Visual SLAM demonstrations using NVIDIA Isaac ROS packages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vslam_node = isaac_vslam_demos.vslam_node:main',
            'vslam_mapper = isaac_vslam_demos.vslam_mapper:main',
            'vslam_tracker = isaac_vslam_demos.vslam_tracker:main',
        ],
    },
)