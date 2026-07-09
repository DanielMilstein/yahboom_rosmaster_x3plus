from setuptools import setup

package_name = 'robot_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/removal.launch.py']),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn'],
    zip_safe=True,
    description='HTTP gateway for robot print-removal jobs',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gateway = robot_gateway.server:main',
        ],
    },
)
