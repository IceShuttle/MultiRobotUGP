from setuptools import find_packages, setup

package_name = 'sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'map.csv']),
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='shivang',
    maintainer_email='shivangso23@iitk.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_publisher = sim.map_publisher:main',
            'map_visualizer = sim.map_visualizer:main',
            'entity_sim = sim.entity_sim:main',
            'solver = sim.solver:main',
        ],
    },
)
