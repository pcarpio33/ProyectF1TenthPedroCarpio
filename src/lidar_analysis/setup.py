from setuptools import setup

package_name = 'lidar_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro',
    maintainer_email='pedro@todo.com',
    description='Nodo de análisis de LiDAR para F1Tenth',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_analysis_node = lidar_analysis.lidar_analysis_node:main',
        ],
    },
)

