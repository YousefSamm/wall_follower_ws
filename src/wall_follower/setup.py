from setuptools import setup
import os
from glob import glob

package_name = 'wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'wall_follower_node=wall_follower.wall_following:main',
        'wall_finder=wall_follower.wall_finder:main',
        'action_server=wall_follower.odom_recorder:main'
        ],
    },
)
