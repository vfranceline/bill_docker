from setuptools import find_packages, setup

package_name = 'tasks'

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
    maintainer='vfran',
    maintainer_email='vitorianascimentomatos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_pose = tasks.goal_pose:main',
            'inspecao = tasks.object_avoidance:main',
        ],
    },
)
