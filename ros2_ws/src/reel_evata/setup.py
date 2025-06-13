from setuptools import find_packages, setup

package_name = 'reel_evata'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akif',
    maintainer_email='akifdlk58svs@gmail.com',
    description='ROS 2 Package for CAN Bus and Brake Control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'OdometerListener = reel_evata.OdometerListener:main',
            'AKS_communication = reel_evata.AKS_communication:main',
            'Aks = reel_evata.Aks:main',
            'stabilThrottle = reel_evata.stabilThrottle:main',
            'velocity_plotter = reel_evata.velocity_plotter:main',
            'tarikutay = reel_evata.tarikutay:main'
        
        ],
    },
)
