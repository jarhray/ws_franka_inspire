from setuptools import find_packages, setup

package_name = 'inspire_executor'

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
    maintainer='inspire_executor',
    maintainer_email='todo@todo.todo',
    description='Inspire hand executor: HandAction to SetAngle1.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspire_executor_node = inspire_executor.inspire_executor_node:main',
        ],
    },
)
