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
    entry_points={
        'console_scripts': [
            'swarm_control = sr_control.swarm_control:main',
            'swarm_control_2 = sr_control.swarm_control_2:main',

        ],
    },
)
