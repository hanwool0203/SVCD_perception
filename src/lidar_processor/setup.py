from setuptools import find_packages, setup

package_name = 'lidar_processor'

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
    maintainer='omen16',
    maintainer_email='hanwool0203@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pc2np_opt_node = lidar_processor.pc2np_optimzer:main',
            'pc2np_opt_kitti = lidar_processor.pc2np_opt_kitti:main',
            'tracker = lidar_processor.tracker:main',
            'test_node = lidar_processor.test:main',
            'marker_tracker_node = lidar_processor.marker_tracker:main',
        ],
    },
)
