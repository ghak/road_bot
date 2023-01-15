from setuptools import setup
import os
import glob

package_name = 'road_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*launch.[pxy][yma]*'))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghak',
    maintainer_email='todo@todo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_bot_bridge = road_bot.road_bot_bridge:main',
            'camera_bridge = road_bot.camera_bridge:main',
            'stereo_vision = road_bot.stereo_vision:main',
        ],
    },
)
