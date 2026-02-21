from setuptools import setup
import os
from glob import glob

package_name = 'plant'

def files_in(*patterns, recursive=False):
    paths = []
    for p in patterns:
        paths.extend(glob(p, recursive=recursive))
    return [f for f in paths if os.path.isfile(f)]

data_files = [
    # ament index
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    # package.xml
    (f'share/{package_name}', ['package.xml']),
    # assets 
    (f'share/{package_name}/urdf',   files_in('urdf/*', 'urdf/**/*', recursive=True)),
    (f'share/{package_name}/launch', files_in('launch/*', 'launch/**/*', recursive=True)),
    (f'share/{package_name}/config', files_in('config/*', 'config/**/*', recursive=True)),
    (f'share/{package_name}/rviz',   files_in('rviz/*', 'rviz/**/*', recursive=True)),
    (f'share/{package_name}/meshes', files_in('meshes/*', 'meshes/**/*', recursive=True)),
]

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],   
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woody',
    maintainer_email='woody@example.com',
    description='Turntable + cameras + Gazebo + RViz (URDF + launch)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_capturer_node = plant.capture_images:main',
        ],
    },
)

