from setuptools import find_packages, setup

package_name = 'action_router'

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
    maintainer='action_router',
    maintainer_email='todo@todo.todo',
    description='Policy action routing with safety and relative/absolute handling.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_router_node = action_router.action_router_node:main',
        ],
    },
)
