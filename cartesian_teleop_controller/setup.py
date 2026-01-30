from setuptools import find_packages, setup

package_name = 'cartesian_teleop_controller'

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
    maintainer='asthrart1',
    maintainer_email='balachandar10301@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                    'ik_controller = cartesian_teleop_controller.ik_controller:main',
                    'joint_space_controller = cartesian_teleop_controller.joint_space_controller:main',
                    'task_space_controller = cartesian_teleop_controller.task_space_ik_control:main',
                    'task_space_teleop = cartesian_teleop_controller.task_space_teleop:main',

        ],
    },
)
