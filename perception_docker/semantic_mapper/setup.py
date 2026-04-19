from setuptools import setup
from glob import glob

package_name = 'semantic_mapper'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='MORE-style scene graph + YOLOv8 for LLM-guided drone search.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector        = semantic_mapper.yolo_detector:main',
            'semantic_mapper_node = semantic_mapper.semantic_mapper_node:main',
            'object_mapper_node   = semantic_mapper.object_mapper_node:main',
            'room_labeler         = semantic_mapper.room_labeler:main',
        ],
    },
)
