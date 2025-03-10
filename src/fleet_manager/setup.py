from setuptools import find_packages, setup

package_name = 'fleet_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ujjwal',
    maintainer_email='ujjwalpatil20@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_manager = fleet_manager.fleet_manager:main',
            'goal_publisher = fleet_manager.goal_publisher:main',
            'service_test = fleet_manager.service_test:main',
            'goal_service_publisher = fleet_manager.goal_service_publisher:main',
            'service_caller = fleet_manager.service_call_example:main',
            'merged_fleet_manager = fleet_manager.merged_fleet_manager:main',
        ],
    },
)
