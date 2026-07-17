from setuptools import find_packages, setup

package_name = 'bluerov2_detection'

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
            'april_tag = bluerov2_detection.ReadAprilTags:main',
            'localization = bluerov2_detection.Localization:main',
            'depth = bluerov2_detection.PressureToDepth:main',
            'SLAM = bluerov2_detection.SLAM:main'
        ],
    },
)
