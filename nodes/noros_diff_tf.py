#!/usr/bin/env python
PKG = 'test_knex_ros'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
sys.path.append('/home/jfstepha/ros_workspace/differential_drive/nodes')


import unittest
import diff_tf
from std_msgs.msg import Float32
from std_msgs.msg import Int16


############################################################################## 
class TestDiffTf(unittest.TestCase):
############################################################################## 
    ############################################################################## 
    def setUp(self):
    ############################################################################## 
        self.test_rate = rospy.get_param("~test_rate",100) 
        rospy.set_param("/diff_tf/ticks_meter",10)
        self.myDiffTf = diff_tf.DiffTf()
        
    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        self.assertEquals(1, 1, "1!=1")
        
    ############################################################################## 
    def test_no_data(self):
    ############################################################################## 
        self.myDiffTf.update()
        self.compare(0,0,0,0,0,"test_no_data")
        
    ############################################################################## 
    def test_drive_1m(self):
    ############################################################################## 
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = 0.0
        self.myDiffTf.right = 0.0
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = self.myDiffTf.ticks_meter
        self.myDiffTf.right = self.myDiffTf.ticks_meter
        self.myDiffTf.update()
        self.compare(1,0,0,1,0,"test_drive_1m")
        
    ############################################################################## 
    def test_drive_m1m(self):
    ############################################################################## 
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = 0.0
        self.myDiffTf.right = 0.0
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = -self.myDiffTf.ticks_meter
        self.myDiffTf.right = -self.myDiffTf.ticks_meter
        self.myDiffTf.update()
        self.compare(-1,0,0,-1,0,"test_drive_m1m")
        
    ############################################################################## 
    def test_rotate_90(self):
    ############################################################################## 
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = 0.0
        self.myDiffTf.right = 0.0
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = self.myDiffTf.base_width * self.myDiffTf.ticks_meter
        self.myDiffTf.right = 0
        self.myDiffTf.update()
        self.compare(0.066,0.103,-1,0.122,-1,"test_drive_1m")
        
    ############################################################################## 
    def test_rotate_m90(self):
    ############################################################################## 
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = 0.0
        self.myDiffTf.right = 0.0
        self.myDiffTf.update()
        rospy.sleep(1)
        self.myDiffTf.left = 0
        self.myDiffTf.right = self.myDiffTf.base_width * self.myDiffTf.ticks_meter
        self.myDiffTf.update()
        self.compare(0.066,-0.103,1,0.122,1,"test_drive_1m")
        
    ############################################################################## 
    def compare(self,x,y,th, dx,dr, message):
    ############################################################################## 
        rospy.loginfo("compare: actuals: x:%0.3f y:%0.3f th:%0.3f dx:%0.3f dr:%0.3f " % (self.myDiffTf.x, self.myDiffTf.y, self.myDiffTf.th, self.myDiffTf.dx, self.myDiffTf.dr))
        self.assertAlmostEqual(self.myDiffTf.x, x,2, "%s: x != %0.3f (%0.3f != %0.3f)" % (message, x, self.myDiffTf.x,x))
        self.assertAlmostEqual(self.myDiffTf.y, y,2, "%s: y != %0.3f (%0.3f != %0.3f)" % (message, y, self.myDiffTf.y,y))
        self.assertAlmostEqual(self.myDiffTf.th, th,2, "%s: th != %0.3f (%0.3f != %0.3f)" % (message, th, self.myDiffTf.th,th))
        self.assertAlmostEqual(self.myDiffTf.dx, dx,2, "%s: dx != %0.3f (%0.3f != %0.3f)" % (message, dx, self.myDiffTf.dx,dx))
        self.assertAlmostEqual(self.myDiffTf.dr, dr,2, "%s: dth != %0.3f (%0.3f != %0.3f)" % (message, dr, self.myDiffTf.dr,dr))
        
        
        
if __name__ == '__main__':    
    import rostest
    rospy.loginfo("-I- noros_diff_tf started")
    rostest.rosrun(PKG, 'test_diff_tf', TestDiffTf, sys.argv) 
    