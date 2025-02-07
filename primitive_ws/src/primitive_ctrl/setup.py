from setuptools import find_packages, setup
from glob import glob

package_name = 'primitive_ctrl'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(where='src',
                           exclude=['test']),       #[package_name],     
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),   #'share/' +
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools',
                      'casadi',     # This is a PIP dependency for the Push controller
                      'numpy',      # This is a PIP dependency for the Push controller
                      'pytransform3d',  # PIP dependency for 3D Transform in Push Ctrl
                     ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'set_param = primitive_ctrl.set_param:main',
                'move3d_test = primitive_ctrl.move3d_test:main',
                'move3d_action_server = primitive_ctrl.move3d_action_server:main',
                'move3d_action_client_test = primitive_ctrl.move3d_action_client_test:main',
                'mover_client_main = primitive_ctrl.mover_client_main:main'
        ],
    },

    zip_safe=True,
    maintainer='abhara13',
    maintainer_email='abhara13@asu.edu',
    description='Contains controller nodes for different motion primitives',
    license='Apache License 2.0',
)
