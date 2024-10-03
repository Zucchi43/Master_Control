from setuptools import find_packages, setup

package_name = 'nmea_pub'

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
    maintainer='zucchi',
    maintainer_email='zucchi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmea_sentence_pub = nmea_pub.nmea_sentence_pub:main',
            'nmea_navsatfix_pub = nmea_pub.nmea_navsatfix_pub:main',
            'nmea_publisher = nmea_pub.SimulaGPSROS2',
        ],
    },
)
