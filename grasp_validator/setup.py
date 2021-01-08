from setuptools import setup

package_name = 'grasp_validator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosi',
    maintainer_email='glenn_tan_from.tp@artc.a-star.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'validator = grasp_validator.grasp_validator:main',
                'depth_checker = grasp_validator.depth_checker:main',
        ],
    },
)
