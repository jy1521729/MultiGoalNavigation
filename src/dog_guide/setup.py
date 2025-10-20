from setuptools import setup
from glob import glob
package_name = 'dog_guide'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/params', glob('params/*')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*')),
        ('share/' + package_name + '/sounds', glob('sounds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bt_creater = dog_guide.bt_creater:main',
            'dog_controller = dog_guide.dog_controller:main',
        ],
    },
)
