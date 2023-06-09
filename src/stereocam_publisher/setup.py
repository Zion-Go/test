from setuptools import setup
import os
from glob import glob

package_name = 'stereocam_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('theimagingsource', 'launch'), 
        glob(os.path.join('launch', '*launch.[py][yaml]*')))
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='slxx5237@gmail.com',
    description='stereo camera publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
