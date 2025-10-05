from setuptools import find_packages, setup

package_name = 'nav_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim_world.launch.py',
            'launch/pipeline_demo.launch.py',
            'launch/rviz_only.launch.py',
            'launch/full_demo.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/waypoints.yaml',
            'config/controller.yaml'
        ]),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pheonix',
    maintainer_email='am2836166@gmail.com',
    description='Smoothing + time trajectory + pure pursuit controller for diff drive',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_server = nav_pipeline.waypoint_server:main',
            'path_smoother = nav_pipeline.path_smoother:main',
            'trajectory_generator = nav_pipeline.trajectory_generator:main',
            'pure_pursuit_controller = nav_pipeline.pure_pursuit_controller:main',

        ],
    },
)
