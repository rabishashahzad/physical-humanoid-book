from setuptools import find_packages, setup

package_name = 'vla_llm_integration'

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
    description='Vision-Language-Action LLM integration package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_planning_node = vla_llm_integration.llm_planning_node:main',
            'command_parser_node = vla_llm_integration.command_parser_node:main',
        ],
    },
)