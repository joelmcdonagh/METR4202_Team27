from setuptools import setup

package_name = 'waypoint_commander'

setup(
    name=package_name,
    version='0.0.1',
    # ⬇️ flat layout: no inner package dir, just modules
    py_modules=['main2', 'exploration'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/exploration.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@uqconnect.edu.au',
    description='Exploration and waypoint control for TurtleBot3 (METR4202)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # ros2 run waypoint_commander exploration
            'exploration = main2:main',
        ],
    },
)
