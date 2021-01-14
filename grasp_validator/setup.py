from setuptools import setup

package_name = 'grasp_validator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tan Jing Peng Glenn',
    maintainer_email='glenn_tan_from.tp@artc.a-star.edu.sg',
    description='ROS2 Package to visually represent grasps and to check depth values.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'validator = grasp_validator.grasp_validator:main',
                'depth_checker = grasp_validator.depth_checker:main',
        ],
    },
)
