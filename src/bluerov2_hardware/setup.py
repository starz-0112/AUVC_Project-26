from setuptools import find_packages, setup

package_name = 'bluerov2_hardware'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_disarm = bluerov2_hardware.ArmDisarm:main',
            'fake_battery = bluerov2_hardware.FakeBattery:main',
            'fake_camera = bluerov2_hardware.FakeCamera:main',
            'lights = bluerov2_hardware.LEDControl:main',
            'mission_timer = bluerov2_hardware.MissionTimer:main',
        ],
    },
)
