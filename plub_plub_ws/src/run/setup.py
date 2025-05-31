from setuptools import setup
from glob import glob
import os

package_name = 'run'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament index entry
        ('share/ament_index/resource_index/packages', 
            [os.path.join('resource', package_name)]),
        # package manifest
        (f'share/{package_name}', ['package.xml']),
        # install all launch scripts
        (f'share/{package_name}/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@domain.com',
    description='Meta-launcher for workspace',
    license='Apache-2.0',
)

