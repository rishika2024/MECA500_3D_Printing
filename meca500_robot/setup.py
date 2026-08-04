import glob
from setuptools import find_packages, setup

package_name = 'mecha500_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[       
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/meshes', glob.glob('meshes/*')),
        ('share/' + package_name + '/meshes_meca500', glob.glob('meshes_meca500/*')),
        ('share/' + package_name + '/meshes_extruder', glob.glob('meshes_extruder/*')),
        ('share/' + package_name + '/meshes_meca500_with_ender3', glob.glob('meshes_meca500_with_ender3/*')),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf.xacro',
                                             'urdf/robot.ros2_control.xacro',
                                             'urdf/meca500.urdf',
                                             'urdf/ender3_extruder.urdf',
                                             'urdf/ender3_environment.urdf',
                                             'urdf/config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rishika2024',
    maintainer_email='berarishika@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
