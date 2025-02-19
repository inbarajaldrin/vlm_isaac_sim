from setuptools import find_packages, setup

package_name = 'intel_cam_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
                                ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaugus11',
    maintainer_email='aaugus11@asu.edu',
    description='Camera code for Intel Camera-based detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intel_jenga_pose_logic = intel_cam_pkg.intel_jenga_pose_logic:main',
            'intel_jenga_pose_topic = intel_cam_pkg.intel_jenga_pose_topic:main'

        ],
    },
)
