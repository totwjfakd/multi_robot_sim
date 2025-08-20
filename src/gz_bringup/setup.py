from setuptools import find_packages, setup

package_name = 'gz_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('lib/' + package_name, ['scripts/odom_to_tf.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanbaek',
    maintainer_email='hanbaek.park@s-innovations.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gz-bringup = gz_bringup.cli:main',
        ],
    },
)
