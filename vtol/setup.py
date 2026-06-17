import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vtol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일들을 설치 디렉토리에 포함시키는 설정
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyujin',
    maintainer_email='a01064857159@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_node = vtol.cam_node:main',
            'precision_land = vtol.precision_land:main',
            'precision_hover = vtol.precision_hover:main',
        ],
    },
)
