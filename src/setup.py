from setuptools import find_packages, setup

package_name = 'cleaner_coverage'

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
    maintainer='project',
    maintainer_email='project@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'coverage = cleaner_coverage.coverage_node:main',
        'coverage_viz = cleaner_coverage.coverage_viz:main',
        'frontier_coverage = cleaner_coverage.frontier_coverage:main',
    ],
},
)
