from setuptools import find_packages, setup
import os

package_name = 'simulation_env'

def package_files(directory):
    """Recursively collect all files under a directory."""
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

launch_files = package_files('launch')
model_files = package_files('models')
rviz_files = package_files('rviz')
urdf_files = package_files('urdf')
world_files = package_files('worlds')
map_files = package_files('maps')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/models', model_files),
        ('share/' + package_name + '/rviz', rviz_files),
        ('share/' + package_name + '/urdf', urdf_files),
        ('share/' + package_name + '/worlds', world_files),
        ('share/' + package_name + '/maps', map_files),
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
        ],
    },
)
