from setuptools import find_packages, setup

package_name = 'arm_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/kinematics.yaml', 'config/collision_objects.yaml']),
        ('share/' + package_name + '/launch', ['launch/planning.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricou',
    maintainer_email='ricou@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_solver_node = arm_planning.ik_solver_node:main',
            'trajectory_planner_node = arm_planning.trajectory_planner_node:main',
            'collision_checker_node = arm_planning.collision_checker_node:main',
            'planning_coordinator_node = arm_planning.planning_coordinator_node:main',
        ],
    },
)
