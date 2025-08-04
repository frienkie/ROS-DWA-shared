from setuptools import setup

package_name = 'shared_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/remote.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='The shared_core package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_node = shared_core.dwa_ros2:main',
        ],
    },
) 