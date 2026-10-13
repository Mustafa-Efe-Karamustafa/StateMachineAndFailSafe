# =============================================================================
# DOSYA : setup.py
# AMAÇ  : Python paketini colcon/ament'e tanıtır: hangi dosyalar nereye kurulacak (launch, world, config)
#         ve `ros2 run sualti_sim_son motion_node` komutunun hangi fonksiyona bağlandığı. Matematik içermez.
# =============================================================================
from glob import glob

from setuptools import find_packages, setup

package_name = 'sualti_sim_son'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    # data_files: (kurulum klasörü, [kaynak dosyalar]). resource/<paket> ament'in paketi bulması için gerekli işaret dosyası.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sualti Takimi',
    maintainer_email='takim@example.com',
    description='Gazebo fizikli su altı aracı: kare ve daire sürüş, PID',
    license='MIT',
    # console_scripts: 'motion_node' komutu = sualti_sim_son/motion_node.py içindeki main()
    entry_points={'console_scripts': ['motion_node = sualti_sim_son.motion_node:main']},
)
