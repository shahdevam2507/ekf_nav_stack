from setuptools import find_packages, setup

package_name = 'global_planner'

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
    maintainer='hiro',
    maintainer_email='jivaldhingra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'astar_planner = global_planner.astar_planner:main',
            'mpc_controller = global_planner.mpc_controller:main',
            'mpc_casadi = global_planner.mpc_casadi:main'
        ],
    },
)
