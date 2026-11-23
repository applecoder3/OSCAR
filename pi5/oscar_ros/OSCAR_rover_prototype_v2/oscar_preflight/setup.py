from setuptools import setup
from glob import glob
package_name = 'oscar_preflight'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='OSCAR team', maintainer_email='maintainer@example.invalid',
      description='OSCAR safe preflight modules', license='Apache-2.0',
      entry_points={'console_scripts': [
          'mock_odometry = oscar_preflight.mock_odometry:main',
          'encoder_monitor = oscar_preflight.encoder_monitor:main',
          'outlet_detector = oscar_preflight.outlet_detector:main',
          'outlet_approach = oscar_preflight.outlet_approach:main',
          'preflight_status = oscar_preflight.preflight_status:main',
          'rear_odometry = oscar_preflight.rear_odometry:main',
          'serial_bridge = oscar_preflight.serial_bridge:main',
      ]})
