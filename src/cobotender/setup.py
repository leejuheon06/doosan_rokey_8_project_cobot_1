from setuptools import setup
from glob import glob
import os

package_name = 'cobotender'


def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        install_path = os.path.join('share', package_name, path)
        paths.append((install_path, files))
    return paths


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

data_files += package_files('templates')
data_files += package_files('static')
data_files += package_files('database')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@example.com',
    description='CoboTender bartender robot UI and Doosan control package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'app = cobotender.app:main',
            'bartender = cobotender.bartender:main',
            'bridge = cobotender.bartender_admin_control_bridge:main',
        ],
    },
)
