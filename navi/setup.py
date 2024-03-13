from setuptools import setup
from setuptools import setup
import os
from glob import glob
package_name = 'navi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yuuuu',
    maintainer_email='kevincolin933@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['navv = navi.navi_node:main',
                            'odom2map = navi.trans_node:main',
                            'testnode = navi.testGoalPose:main',
                            'map_saver_node= navi.map_saver_node:main',
                            'action_nav= navi.action_navi_node:main',
        ],
    },

    
)
