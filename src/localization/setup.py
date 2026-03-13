from setuptools import setup
from glob import glob

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nihad',
    maintainer_email='nihadjifri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion = localization.sensor_fusion:main',
            'slam = localization.slam:main',
            'slam_evaluator = localization.slam_evaluator:main',
            'ekf_slam = localization.ekf_slam:main',
        ],
    },
)
