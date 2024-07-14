from setuptools import setup

package_name = 'test_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juyong',
    maintainer_email='juyong3393@snu.ac.kr',
    description='test nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_speaker = test_nodes.obstacle_speaker:main',
            'gimbal_test = test_nodes.gimbal_test:main',
            'mc_test_00 = test_nodes.mc_test_00_arm:main',
            'mc_test_01 = test_nodes.mc_test_01_takeoff_and_land:main',
            'mc_test_02 = test_nodes.mc_test_02_mc_square:main',
            'yolo_test_01 = test_nodes.yolo_test_01:main',
            'yolo_test_02 = test_nodes.yolo_test_02_go_straight:main',
            'yolo_test_03 = test_nodes.yolo_test_03_and_turn_left:main',
            'yolo_test_04 = test_nodes.yolo_test_04_gimbal:main',
        ],
    },
)
