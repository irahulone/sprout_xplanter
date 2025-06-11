from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'planter_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    #    (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rover2-i',
    maintainer_email='rover2-i@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = planter_module.motor_controller:main',
            'motor_controller_test = planter_module.motor_controller_test:main',
            'planter_module_server = planter_module.planter_module_server:main',
            'planter_module_client = planter_module.planter_module_client:main'
        ],
    },
)
