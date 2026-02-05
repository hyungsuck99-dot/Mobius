from setuptools import setup
package_name = 'tcp_plc_reader'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tcp_plc.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='TCP PLC reader → ROS2 topics (box/gap detector)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tcp_plc_node = tcp_plc_reader.tcp_plc_node:main',
        ],
    },
)

