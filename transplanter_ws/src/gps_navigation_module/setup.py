from setuptools import find_packages, setup

package_name = 'gps_navigation_module'

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
    maintainer='rsl',
    maintainer_email='rsl@scu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_action_server = gps_navigation_module.waypoint_action_server:main',
            'waypoint_action_client = gps_navigation_module.waypoint_action_client:main',
            'waypoint_controller = gps_navigation_module.waypoint_controller:main'
        ],
    },
)
