from setuptools import setup

package_name = 'mecanum_drive'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/mecanum_launch.py']),
        ('share/' + package_name + '/action', ['action/GoToPose.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Dudas',
    maintainer_email='david.dudas@outlook.com',
    description='ROS 2 port of the mecanum drive package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_drive_controller = mecanum_drive.mecanum_drive_controller:main',
            'mecanum_drive_odometry = mecanum_drive.mecanum_drive_odometry:main',
        ],
    },
)
