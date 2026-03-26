from setuptools import find_packages, setup

package_name = 'fr3_franky_executor'

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
    maintainer='fr3_franky_executor',
    maintainer_email='todo@todo.todo',
    description='FR3 executor framework with mockable franky backend.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fr3_franky_executor_node = fr3_franky_executor.fr3_franky_executor_node:main',
        ],
    },
)
