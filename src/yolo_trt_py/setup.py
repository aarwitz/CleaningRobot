from setuptools import setup

package_name = 'yolo_trt_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taylor',
    maintainer_email='taylor@localhost',
    description='Pure Python TensorRT YOLOv8 inference node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_trt_node = yolo_trt_py.yolo_trt_node:main',
        ],
    },
)
