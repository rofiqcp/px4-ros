"""Offline mission geodesy and VESC kinematics tests (no ROS/serial dependencies)."""
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/px4_mission_foxglove'))
sys.path.insert(0,str(ROOT/'src/px4_vesc_interface'))
from px4_mission_foxglove.math_utils import waypoint,xy,preview
from px4_vesc_interface.kinematics import drive

class GuidanceTests(unittest.TestCase):
    def test_global_waypoint(self):
        item=SimpleNamespace(x=-78000000,y=1100000000,command=16,frame=3)
        self.assertTrue(waypoint(item))
        item.command=178
        self.assertFalse(waypoint(item))
    def test_mission_coordinates(self):
        a,b=xy(-7.8,110.1,-7.8,110.1)
        self.assertAlmostEqual(a,0.);self.assertAlmostEqual(b,0.)
        self.assertGreater(xy(-7.8,110.10001,-7.8,110.1)[0],1.)
    def test_heading_and_clamp(self):
        speed,yaw,reason=preview((0.,0.),(5.,0.),0.,1.,0.,cruise=.3)
        self.assertEqual(reason,'PREVIEW_ONLY');self.assertAlmostEqual(speed,.3)
        self.assertAlmostEqual(yaw,0.)
        speed,yaw,_=preview((0.,0.),(-5.,0.),0.,1.,0.,cruise=.3)
        self.assertEqual(speed,0.)
    def test_stop_and_geometry(self):
        self.assertEqual(drive(0.,.2,.8,.7,.16)[-1],'STOP')
        self.assertEqual(drive(.2,.1,0.,.7,.16)[-1],'GEOMETRY_UNVERIFIED')
    def test_differential_projection(self):
        angle,left,right,lr,rr,state=drive(.2,.1,.8,.7,.16)
        self.assertEqual(state,'PREVIEW_ONLY')
        self.assertGreater(right,left)
        self.assertGreater(rr,lr)
        self.assertLessEqual(abs(angle),.45)
    def test_nonfinite_fails_closed(self):
        self.assertEqual(drive(float('nan'),0.,.8,.7,.16)[-1],'NONFINITE')
        self.assertEqual(preview((0.,0.),(1.,0.),float('nan'),.3,0.)[-1],'INVALID_NUMERIC_INPUT')

if __name__=='__main__':unittest.main()
