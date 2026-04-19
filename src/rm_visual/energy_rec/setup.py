from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'energy_rec'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加下面这一行：将 energy_rec 目录下的所有 .pt 文件安装到 share/energy_rec/
        (os.path.join('share', package_name), glob('energy_rec/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='3192937647@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'energy_rec_node = energy_rec.energy_rec:main',
        ],
    },
)