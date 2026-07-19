from setuptools import find_packages, setup

package_name = 'ekfslam'

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
    maintainer='rover',
    maintainer_email='rover@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekfnode = ekfslam.ekfnode:main',
            'ekfviz = ekfslam.ekfviz:main',
            'ekflocnode = ekfslam.ekflocnode:main',
            'ekflocviz = ekfslam.ekflocviz:main',
        ],
    },
)
