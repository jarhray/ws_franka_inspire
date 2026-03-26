from setuptools import setup
from glob import glob
import os

package_name = 'inspire_hand_modbus_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inspire001',
    maintainer_email='inspire001@example.com',
    description='A ROS 2 package for inspire hand modbus control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handcontrol_topic_publisher_modbus = inspire_hand_modbus_ros2.handcontrol_topic_publisher_modbus:main',
            'handcontrol_topic_subscriber_modbus = inspire_hand_modbus_ros2.handcontrol_topic_subscriber_modbus:main',
            'inspire_hand_modbus_topic = inspire_hand_modbus_ros2.inspire_hand_modbus_topic:main',
        ],
    },
)

