from setuptools import setup

package_name = 'pointcloud_to_ply'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
      ('share/' + package_name, ['package.xml']),
      ('share/' + package_name + '/launch', ['launch/pointcloud_to_ply.launch.xml']),
      ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
      ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li9i',
    maintainer_email='alexandros.filotheou@gmail.com',
    description='Subscribe to a PointCloud2 topic, reconstruct a mesh, and save to OBJ/PLY',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
      'console_scripts': [
        'pointcloud_to_ply_node = pointcloud_to_ply.pointcloud_to_ply_node:main',
        ],
      },
    )
