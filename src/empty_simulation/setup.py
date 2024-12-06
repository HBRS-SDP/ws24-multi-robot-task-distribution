from setuptools import find_packages, setup

package_name = 'empty_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/empty_world.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='behrouz',
    maintainer_email='behroozghamkhar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

