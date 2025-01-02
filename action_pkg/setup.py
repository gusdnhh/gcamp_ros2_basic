from setuptools import find_packages, setup

package_name = 'action_pkg'

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
    maintainer='gusdnhh',
    maintainer_email='gusdnhh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_subscriber = action_pkg.image_sub:main',
            'odom_subscriber = action_pkg.odom_sub:main',
            'maze_action_server = action_pkg.maze_action_server:main',
            'maze_action_client = action_pkg.maze_action_client:main',
            'turnning_server = action_pkg.turnning_server:main',
        ],
    },
)
