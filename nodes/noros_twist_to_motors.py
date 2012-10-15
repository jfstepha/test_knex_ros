#!/usr/bin/env python
PKG = 'test_knex_ros'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
sys.path.append('/home/jfstepha/ros_workspace/differential_drive/nodes')

import unittest
import twist_to_motors
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist


############################################################################## 
class TestNoRosTwistToMotors(unittest.TestCase):
############################################################################## 
    ############################################################################## 
    def setUp(self):
    ############################################################################## 
        self.test_rate = rospy.get_param("~test_rate",100) 
        self.myTwist = twist_to_motors.TwistToMotors()
        
    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        self.assertEquals(1, 1, "1!=1")
        
    
    ############################################################################## 
    def test_x1(self):
    ############################################################################## 
        self.sendmsg(1,0,0)
        self.assertEquals(self.myTwist.left, 1, "test_x1 left != 1 (%0.3f != 1)" % self.myTwist.left)
        self.assertEquals(self.myTwist.right, 1, "test_x1 right != 1 (%0.3f != 1)" % self.myTwist.right)
        
    ############################################################################## 
    def test_x2(self):
    ############################################################################## 
        self.sendmsg(2,0,0)
        self.assertEquals(self.myTwist.left, 2, "test_x2 left != 1 (%0.3f != 2)" % self.myTwist.left)
        self.assertEquals(self.myTwist.right, 2, "test_x2 right != 1 (%0.3f != 2)" % self.myTwist.right)
        
    ############################################################################## 
    def test_xm1(self):
    ############################################################################## 
        self.sendmsg(-1,0,0)
        self.assertEquals(self.myTwist.left, -1, "test_xm1 left != -1 (%0.3f != -1)" % self.myTwist.left)
        self.assertEquals(self.myTwist.right, -1, "test_xm1 right != -1 (%0.3f != -1)" % self.myTwist.right)
        
    ############################################################################## 
    def test_r1(self):
    ############################################################################## 
        self.sendmsg(0,1,0)
        self.assertEquals(self.myTwist.left, -0.1, "test_r1 left != -0.1 (%0.3f != -0.1)" % self.myTwist.left)
        self.assertEquals(self.myTwist.right, 0.1, "test_r1 right != 0.1 (%0.3f != 0.1)" % self.myTwist.right)
        
    ############################################################################## 
    def test_rm1(self):
    ############################################################################## 
        self.sendmsg(0,-1,0)
        self.assertEquals(self.myTwist.left,  0.1, "test_r1 left != 0.1 (%0.3f != 0.1)" % self.myTwist.left)
        self.assertEquals(self.myTwist.right,-0.1, "test_r1 right != -0.1 (%0.3f != -0.1)" % self.myTwist.right)
        
    ############################################################################## 
    def test_xr1(self):
    ############################################################################## 
        self.sendmsg(1,1,0)
        self.assertEquals(self.myTwist.left, 0.9, "test_xr1 left != 0.9 (%0.3f != 0.9)" % self.myTwist.left)
        self.assertEquals(self.myTwist.right, 1.1, "test_xr1 right != 1.1 (%0.3f != 1.1)" % self.myTwist.right)
         
    ############################################################################## 
    def test_base_0p5(self):
    ############################################################################## 
        old_w = self.myTwist.w
        self.myTwist.w = 0.5
        self.sendmsg(1,1,0)
        self.assertEquals(self.myTwist.left, 0.75, "test_base_0p5 left != 0.75 (%0.3f != 0.75)" % self.myTwist.left)
        self.assertEquals(self.myTwist.right, 1.25, "test_base_0p5 right != 1.25 (%0.3f != 1.25)" % self.myTwist.right)
        
       
    ############################################################################## 
    def sendmsg(self, x, r, y):
    ############################################################################## 
        msg = Twist();
        rospy.loginfo("-D- sendmsg: x=%0.3f r=%0.3f y=%0.3f" %(x,r,y))
        msg.linear.x = x;
        msg.angular.z = r;
        msg.linear.y = y;
        self.myTwist.twistCallback(msg)
        self.myTwist.spinOnce()
        
############################################################################## 
if __name__ == '__main__':    
############################################################################## 
    import rostest
    rospy.loginfo("-I- noros_twist_to_motors started")
    rostest.rosrun(PKG, 'noros_twist_to_motors', TestNoRosTwistToMotors, sys.argv) 
    