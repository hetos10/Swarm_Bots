from setuptools import find_packages, setup

package_name = 'sr_control'

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
    maintainer='hetos_10',
    maintainer_email='hetchauhan22@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_controller = sr_control.mecanum_controller:main',
            'arm_controller = sr_control.arm_controller:main',
            'piston_controller = sr_control.piston_controller:main',
        ],
    },
)
