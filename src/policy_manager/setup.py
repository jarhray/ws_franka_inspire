from setuptools import find_packages, setup

package_name = 'policy_manager'

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
    maintainer='policy_manager',
    maintainer_email='todo@todo.todo',
    description='Policy type switcher with dummy WholeBodyAction output.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_manager_node = policy_manager.policy_manager_node:main',
        ],
    },
)
