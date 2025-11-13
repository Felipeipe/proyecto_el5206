from setuptools import find_packages, setup

package_name = 'football_player'

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
    maintainer='ijo_el_estiven',
    maintainer_email='phillipe.joos@ug.uchile.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follow_and_avoid = football_player.follow_and_avoid:main',
            'detector = football_player.yolo_detector_node:main'
        ],
    },
)
