import os
from setuptools import setup

package_name = 'example'

# launch/example.launch.py 파일 존재 여부 확인 후 data_files 추가
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# 이미지 폴더 구조 기준 실제 파일명(example.launch.py) 반영
launch_file_path = os.path.join('launch', 'example.launch.py')
if os.path.exists(launch_file_path):
    data_files.append(('share/' + package_name + '/launch', [launch_file_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'requests', 'flask', 'flask-socketio'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='LiDAR, IMU and GPS Integration Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = example.talker:main',
            'listener = example.listener:main',
            'gps_pub = example.gps_publisher_node:main',
            'gps_map_node = example.gps_map_node:main',
        ],
    },
)