from setuptools import find_packages, setup

package_name = 'order_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'flask', 'robot_interfaces', 'rosbridge-websocket'],
    zip_safe=True,
    maintainer='ayushi',
    maintainer_email='ayushiarora206@gmail.com',
    description='Order Publisher Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_publisher = order_publisher.order_publisher:main',
            'web_server = order_publisher.web_server:main',
        ],
    },
)
