from setuptools import setup
pkg='px4_vesc_interface'
setup(name=pkg,version='0.1.0',packages=[pkg],
  data_files=[
     ('share/ament_index/resource_index/packages',['resource/'+pkg]),
     ('share/'+pkg,['package.xml']),
     ('share/'+pkg+'/launch',['launch/vesc_preview.launch.py']),
     ('share/'+pkg+'/config',['config/vesc_interface.yaml']),
  ],install_requires=['setuptools'],zip_safe=True,
  maintainer='PX4 ROS',maintainer_email='developer@example.com',
  description='Guarded dual-VESC virtual Ackermann kinematics; outputs disabled',
  license='Apache-2.0',
  entry_points={'console_scripts':['vesc_preview=px4_vesc_interface.vesc_bridge:main']})
