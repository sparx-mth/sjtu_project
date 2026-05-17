from setuptools import setup
from glob import glob

package_name = 'room_search'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='room_search: nav -> rotate -> approach -> land orchestrator.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'room_search_orchestrator_node = '
            'room_search.room_search_orchestrator_node:main',
        ],
    },
)
