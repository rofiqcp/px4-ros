from setuptools import setup
pkg='px4_mission_foxglove'
setup(name=pkg,version='0.1.0',packages=[pkg],
    data_files=[
      ('share/ament_index/resource_index/packages',['resource/'+pkg]),
      ('share/'+pkg,['package.xml']),
      ('share/'+pkg+'/launch',['launch/mission_foxglove.launch.py']),
      ('share/'+pkg+'/config',['config/mission_observer.yaml'])
    ],install_requires=['setuptools'],zip_safe=True,
    maintainer='PX4 ROS',maintainer_email='developer@example.com',
    description='Read-only PX4 mission viewer and wall-follower preview',license='Apache-2.0',
    entry_points={'console_scripts':['mission_observer=px4_mission_foxglove.observer:main']})
