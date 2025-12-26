from setuptools import find_packages, setup

package_name = 'vla_speech_recognition'

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
    description='Vision-Language-Action speech recognition package using OpenAI Whisper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_node = vla_speech_recognition.whisper_node:main',
            'audio_capture_node = vla_speech_recognition.audio_capture_node:main',
        ],
    },
)