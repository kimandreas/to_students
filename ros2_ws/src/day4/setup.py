from setuptools import find_packages, setup

package_name = 'day4'

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
            'depth_checker = day4.4_5_a_depth_checker:main',
            'depth_to_3d = day4.4_5_b_depth_to_3d:main',
            # 'depth_to_goal = day3.4_5_c_depth_to_nav_goal:main',
            # 'nav_to_person = day3.4_5_d_nav_to_person:main',
        ]
    },
)
