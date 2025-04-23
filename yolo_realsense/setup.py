from setuptools import setup

package_name = 'yolo_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_node.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iras',
    maintainer_email='iras@todo.todo',
    description='YOLO + RealSense + AprilTag Integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_realsense = yolo_realsense.yolo_realsense:main',
            'move_to_target = yolo_realsense.move_to_target:main',
        ],
    },
)

