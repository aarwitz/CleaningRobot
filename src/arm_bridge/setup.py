from setuptools import setup, find_packages

package_name = 'arm_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Stub arm controller interface',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arm_bridge_node = arm_bridge.arm_bridge_node:main',
        ],
    },
)
