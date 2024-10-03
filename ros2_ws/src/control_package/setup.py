from setuptools import find_packages, setup

package_name = 'control_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/get_speed.py']),
        ('lib/' + package_name, [package_name+'/get_steer.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zucchi',
    maintainer_email='zucchi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control_package.control_node:main',
            'ss_pub = control_package.ss_pub:main',
            'gpsfollow = control_package.gpsfollow:main',
        ],
    },
)
