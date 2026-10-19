"""Calibration dashboard correctness, without PX4 or hardware."""
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/px4_calibration_web'))
from px4_calibration_web.readiness import PARAMS, CALIBRATIONS, blockers

class CalibrationReadinessTests(unittest.TestCase):
    def test_zero_airframe_is_blocking(self):
        result=blockers({'connected':True,'mav_state':0,'gps_valid':False,'local_valid':False},
                        {'SYS_AUTOSTART':{'value':0},'COM_RC_IN_MODE':{'value':3}})
        self.assertIn('AIRFRAME',{p[0] for p in result})
        self.assertIn('UNINITIALIZED',{p[0] for p in result})
    def test_ros_manual_setting_is_not_rc_requirement(self):
        result=blockers({'connected':True,'mav_state':3,'gps_valid':True,'local_valid':True},
                        {'SYS_AUTOSTART':{'value':51000},'COM_RC_IN_MODE':{'value':1}})
        self.assertNotIn('MANUAL_SOURCE',{p[0] for p in result})
    def test_no_manual_input_disabled_requires_other_estop(self):
        result=blockers({'connected':True,'mav_state':3,'gps_valid':True,'local_valid':True},
                        {'SYS_AUTOSTART':{'value':50000},'COM_RC_IN_MODE':{'value':4}})
        self.assertNotIn('MANUAL_SOURCE',{p[0] for p in result})
    def test_calibration_is_only_sensor_commands(self):
        self.assertEqual(set(CALIBRATIONS),{'gyro','accel','compass','level'})
        for item in CALIBRATIONS.values():
            v=item['params']
            self.assertEqual(len(v),7)
            self.assertEqual(sum(bool(x) for x in v),1)
    def test_readonly_param_name_list(self):
        self.assertIn('SYS_AUTOSTART',PARAMS)
        self.assertIn('COM_RC_IN_MODE',PARAMS)
        self.assertIn('CAL_MAG0_ID',PARAMS)
    def test_no_link_blocks(self):
        self.assertEqual(blockers({'connected':False},{}),[('LINK','Tidak ada heartbeat MAVLink valid')])

if __name__=='__main__':unittest.main()
