from setuptools import find_packages, setup

package_name = 'task_management'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/RobotStatus.msg']),
        ('share/' + package_name + '/srv', ['srv/Database.srv']),
        ('share/' + package_name + '/srv', ['srv/InventoryUpdate.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayushi',
    maintainer_email='ayushiarora206@gmail.com',
    description='Task Management package for warehouse robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = task_management.task_manager_node:main',
            'goal_publisher = task_management.goal_publisher:main',
            'goal_publisher_1 = task_management.goal_publisher_1:main',
            'data_base = task_management.data_base:main',
        ],
    },
)

