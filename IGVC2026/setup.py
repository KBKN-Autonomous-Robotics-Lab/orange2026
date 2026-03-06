from setuptools import find_packages, setup

package_name = 'IGVC2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'igvc_detection = IGVC2026.igvc_detection:main',
            'igvc2026_detection = IGVC2026.igvc2026_detection:main',
            'igvc_control = IGVC2026.igvc_control:main',
            'human_vest_display = IGVC2026.human_vest_display:main',
            'tire_display = IGVC2026.tire_display:main',
            'stop_sign_display = IGVC2026.stop_sign_display:main',
            'stop_flag = IGVC2026.stop_flag:main',

        ],
    },
)
