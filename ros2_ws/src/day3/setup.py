from setuptools import find_packages, setup

package_name = 'day3'

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
    maintainer='rokey-kim',
    maintainer_email='rokey-kim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_image = day3.3_1_a_capture_image:main',
            'cont_cap_image = day3.3_1_b_cont_capture_image:main',
            'det_obj = day3.3_4_a_yolov8_obj_det:main',
            'det_obj_thread = day3.3_4_b_yolov8_obj_det_thread:main',
            'det_obj_track = day3.3_4_c_yolov8_obj_det_track:main',
            'depth_checker = day3.3_5_a_depth_checker:main',
            'depth_to_3d = day3.3_5_b_depth_to_3d:main',
            'depth_to_goal = day3.3_5_c_depth_to_nav_goal:main',
            'nav_to_person = day3.3_5_d_nav_to_person:main',
        ],
    },
)
