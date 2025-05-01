from setuptools import find_packages, setup

package_name = 'day2'

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
            'pub_image = day2.2_6_a_image_publisher:main',
            'show_image = day2.2_6_b_image_subscriber:main',
            'pub_data = day2.2_6_c_data_publisher:main',
            'show_data = day2.2_6_d_data_subscriber:main',
            'pub_yolo = day2.2_6_e_yolo_publisher:main',
            'show_yolo = day2.2_6_f_yolo_subscriber:main',
        ],
    },
)
