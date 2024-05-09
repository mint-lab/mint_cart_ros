from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mint_cart'
SHARE_DIR = os.path.join("share", package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*"))),
        (os.path.join(SHARE_DIR, "scripts"), glob(os.path.join("scripts", "*")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nrf-mint',
    maintainer_email='nrf-mint@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'barometer_data_logger = mint_cart.barometer_data_logger.atmpress_to_csv:main'
        ],
    },
)
