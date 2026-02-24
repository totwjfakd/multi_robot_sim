import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gz_bringup'
pkg_share = 'share/' + package_name


def _collect_data_files(source_dir, install_prefix):
    """디렉토리를 재귀적으로 탐색하여 data_files 엔트리 생성 (상대 경로)."""
    entries = []
    for root, dirs, files in os.walk(source_dir):
        if files:
            rel = os.path.relpath(root, source_dir)
            dest = install_prefix if rel == '.' else os.path.join(install_prefix, rel)
            entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


# 워크스페이스 루트 (상대 경로: setup.py 기준 ../../)
_ws_rel = os.path.join('..', '..')

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (pkg_share, ['package.xml']),
    (pkg_share + '/launch', glob('launch/*.py')),
    ('lib/' + package_name, ['scripts/odom_to_tf.py']),
    ('lib/' + package_name, ['scripts/gz_spawner.py']),
    ('lib/' + package_name, ['scripts/scan_merger.py']),
    # 리소스 파일
    (pkg_share + '/worlds', glob('worlds/*')),
    (pkg_share + '/urdf', glob('urdf/*')),
]
# models 디렉토리 (하위 폴더 포함)
data_files += _collect_data_files('models', pkg_share + '/models')
# maps 디렉토리 (워크스페이스 루트의 maps/)
data_files += _collect_data_files(os.path.join(_ws_rel, 'maps'), pkg_share + '/maps')
# visualization.rviz (워크스페이스 루트)
_rviz_rel = os.path.join(_ws_rel, 'visualization.rviz')
if os.path.isfile(_rviz_rel):
    data_files.append((pkg_share + '/rviz', [_rviz_rel]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hanbaek',
    maintainer_email='hanbaek.park@s-innovations.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gz-bringup = gz_bringup.cli:main',
        ],
    },
)
