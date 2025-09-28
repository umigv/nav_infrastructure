from setuptools import find_packages, setup

package_name = 'path_tracking'

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
    maintainer='arv',
    maintainer_email='annanova@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit_lookahead = path_tracking.pure_pursuit_lookahead:main',
            'pure_pursuit_controller = path_tracking.pure_pursuit_controller:main'
        ],
    },
)
