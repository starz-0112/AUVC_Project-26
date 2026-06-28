from setuptools import find_packages, setup

package_name = 'bluerov2_behavior'

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
    maintainer='zoeguo',
    maintainer_email='zguo011235@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lights = bluerov2_behavior.FlashyLights:main',
            'movement = bluerov2_behavior.MainMovement:main',
            'manual_control = bluerov2_behavior.ManualNext:main'
        ],
    },
)
