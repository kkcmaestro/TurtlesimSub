from setuptools import find_packages, setup

package_name = 'pid_controller'

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
    maintainer='kkcmaestro',
    maintainer_email='kkcmaestro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller_node = pid_controller.pid_controller:main',
            'circle_node = pid_controller.Circles:main',
            'deceleration_node = pid_controller.Deceleration_node:main',
            'police_turtle_node = pid_controller.Police_turtle:main',
            'police_turtle_node2 = pid_controller.Police_turle:main',
            'pid_grid_node = pid_controller.Grid_code:main',
            'rt_pt_node = pid_controller.RT_PT_node:main',
            'rt_pt_noise_node = pid_controller.PT_RT_noisy_data:main',
            'half_vel_node = pid_controller.PT_RT_half_vel:main',
            'spawn_node = pid_controller.Spawn:main'

        ],
    },
)
