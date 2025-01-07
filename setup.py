import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'a24_mmwave'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob(os.path.join('urdf', '*'))),
        # Include mesh files
        (os.path.join('share', package_name, 'meshes'),
         glob(os.path.join('meshes', '*'))),
        # Include rviz config files
        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*'))),
        # Include map files
        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*'))),
        # Include a24_mmwave files
        (os.path.join('share', package_name, 'a24_mmwave'),
         glob(os.path.join('a24_mmwave', '*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='the-hassan-shahzad',
    maintainer_email='hshahzad2005108277@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'remapper = a24_mmwave.remapper:main',
            'ang_to_pwm = a24_mmwave.ang_to_pwm:main',
            'translator = a24_mmwave.translator:main'
        ],
    },
)
