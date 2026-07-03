from setuptools import setup

package_name = 'depth_self_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aaron Horowitz',
    maintainer_email='aaronhorowitz97@gmail.com',
    description='Self-body removal from depth images before nvblox.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_self_filter_node = depth_self_filter.depth_cutoff_node:main',
            'arm_depth_mask_node = depth_self_filter.arm_depth_mask_node:main',
        ],
    },
)
