"""Conservative health checks; nothing in this module arms or calibrates PX4."""
PARAM_GROUPS = {
 'Airframe & sistem': (
  'SYS_AUTOSTART','SYS_AUTOCONFIG','COM_ARMABLE','COM_ARM_WO_GPS',
  'COM_ARM_MIS_REQ','COM_ARM_SDCARD','COM_ARM_BAT_MIN'),
 'Tanpa receiver RC / ROS': (
  'COM_RC_IN_MODE','COM_RC_LOSS_T','NAV_RCL_ACT','NAV_DLL_ACT','COM_DLL_EXCEPT'),
 'Gyroscope & accelerometer': (
  'CAL_GYRO0_ID','CAL_GYRO1_ID','CAL_GYRO2_ID',
  'CAL_ACC0_ID','CAL_ACC1_ID','CAL_ACC2_ID',
  'SENS_BOARD_ROT','SENS_BOARD_X_OFF','SENS_BOARD_Y_OFF','SENS_BOARD_Z_OFF'),
 'Compass & GPS': (
  'CAL_MAG0_ID','CAL_MAG1_ID','CAL_MAG2_ID',
  'GPS_1_CONFIG','EKF2_GPS_CTRL','EKF2_EV_CTRL',
  'COM_POS_FS_EPH'),
}
PARAMS = tuple(dict.fromkeys(x for group in PARAM_GROUPS.values() for x in group))
CALIBRATIONS = {
 'gyro': {'label':'Gyroscope','params':[1.,0.,0.,0.,0.,0.,0.],
          'steps':['Letakkan Cube pada permukaan diam','Jangan menyentuh atau menggeser selama kalibrasi',
                   'Pantau pesan kemajuan PX4 hingga selesai']},
 'accel': {'label':'Accelerometer 6 sisi','params':[0.,0.,0.,0.,1.,0.,0.],
           'steps':['Siapkan enam orientasi kendaraan','Pegang pada tiap sisi sesuai petunjuk PX4',
                    'Tunggu konfirmasi tiap posisi']},
 'compass': {'label':'Magnetometer / kompas','params':[0.,1.,0.,0.,0.,0.,0.],
             'steps':['Jauhkan dari besi, magnet, dan kabel arus tinggi',
                      'Putar pelan mengikuti petunjuk PX4','Pantau pesan selesai']},
 'level': {'label':'Level horizon','params':[0.,0.,0.,0.,2.,0.,0.],
           'steps':['Tempatkan rover benar-benar mendatar',
                    'Jangan gerakkan saat kalibrasi level','Tunggu ACK dan pesan konfirmasi']},
}

def blockers(status, parameters):
    """Diagnostic hints only; authoritative prearm result belongs to PX4."""
    reasons = []
    if not status.get('connected'):
        reasons.append(('LINK','Tidak ada heartbeat MAVLink valid'))
        return reasons
    auto = parameters.get('SYS_AUTOSTART',{}).get('value')
    if auto == 0:
        reasons.append(('AIRFRAME','SYS_AUTOSTART=0: pilih Generic Rover sesuai chassis dahulu'))
    if status.get('mav_state') == 0:
        reasons.append(('UNINITIALIZED','PX4 melaporkan MAV_STATE_UNINIT'))
    rc = parameters.get('COM_RC_IN_MODE',{}).get('value')
    if rc in (0,2,3,5,6,7,8) and not status.get('rc_telemetry_fresh',False):
        reasons.append(('MANUAL_SOURCE','Tidak ada RC/joystick; tinjau COM_RC_IN_MODE sebelum mode manual'))
    if not status.get('gps_valid'):
        reasons.append(('GPS','GPS belum valid: misi global belum dapat dijalankan'))
    if not status.get('local_valid'):
        reasons.append(('LOCAL_POSITION','Estimator posisi lokal belum valid'))
    return reasons
