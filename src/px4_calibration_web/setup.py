from setuptools import setup
p='px4_calibration_web'
setup(name=p,version='0.1.0',packages=[p],
 data_files=[('share/ament_index/resource_index/packages',['resource/'+p]),
 ('share/'+p,['package.xml']),
 ('share/'+p+'/launch',['launch/calibration_web.launch.py']),
 ('share/'+p+'/config',['config/calibration_web.yaml']),
 ('share/'+p+'/webui',['webui/index.html'])],
 install_requires=['setuptools'],zip_safe=True,
 maintainer='PX4 ROS',maintainer_email='developer@example.com',
 description='Local PX4 calibration and pre-arm web console',license='Apache-2.0',
 entry_points={'console_scripts':['calibration_web=px4_calibration_web.server:main']})
