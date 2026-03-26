from setuptools import find_packages, setup

package_name = 'experiment_logger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='experiment_logger',
    maintainer_email='todo@todo.todo',
    description='JSONL logger for robot observation/action/status/error topics.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment_logger_node = experiment_logger.experiment_logger_node:main',
        ],
    },
)
