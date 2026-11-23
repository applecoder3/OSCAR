from setuptools import find_packages, setup

package_name = 'rover_navigation'

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
    maintainer='wnerronnie',
    maintainer_email='redskinsdbest03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'nav_goal = rover_navigation.nav_goal:main',
        'waypoint_nav = rover_navigation.waypoint_nav:main',
        'cmd_vel_uart = rover_navigation.cmd_vel_uart:main',
    ],
},
)
