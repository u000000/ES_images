from setuptools import find_packages, setup

package_name = 'nut_position_orientation'

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
    maintainer='adam',
    maintainer_email='adasko99a@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nut_position_orientation = nut_position_orientation.nut_position_orientation:main',
            'test_publish = nut_position_orientation.test_publish_image:main'
        ],
    },
)
