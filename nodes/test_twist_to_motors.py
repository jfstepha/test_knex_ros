#!/usr/bin/env python
PKG = 'test_knex_ros'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest
import threading
from optparse import OptionParser

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

MSG_DELAY = 0.2


############################################################################## 
############################################################################## 
class TestTwistToMotors(unittest.TestCase):
############################################################################## 
############################################################################## 
    # def setUp(self):
    ############################################################################## 
    def __init__(self, *args):
    ############################################################################## 
        super(TestTwistToMotors, self).__init__(*args)
        
        
    ############################################################################## 
    def setUp(self):
    ############################################################################## 
        rospy.init_node("test_twist_to_motors")
        
        self.twist_pub = rospy.Publisher("twist", Twist)
        rospy.Subscriber("lwheel_vtarget", Float32, self.lmotor_callback)
        self.lmsg_count = 0
        self.rmsg_count = 0
        rospy.Subscriber("rwheel_vtarget", Float32, self.rmotor_callback)
        rospy.sleep(MSG_DELAY)
        self.base_width = rospy.get_param("/twist_to_motors/base_width",0.2)
            
    ############################################################################## 
    def sendmsgs(self, xmsgs, rmsgs, rate):
    ############################################################################## 
        r = rospy.Rate(rate)
        rospy.loginfo("-D- sendmsgs: sending (%s,%s" % (str(xmsgs), str(rmsgs)))
        for i in range(len(xmsgs)):
            twist = Twist()
            twist.linear.x = xmsgs[i]
            twist.angular.z = rmsgs[i]
            
            rospy.loginfo("-D- sendmsgs publishing %s" % str(twist)) 
            self.twist_pub.publish(twist)
            r.sleep()
            
        
    ############################################################################## 
    def lmotor_callback(self, msg):
    ############################################################################## 
        rospy.loginfo("-D- lmotor received %0.3f" % msg.data)
        self.latest_lmotor = msg.data
        self.time_lmotor = rospy.Time.now()
        self.lmsg_count += 1
        
    ############################################################################## 
    def rmotor_callback(self, msg):
    ############################################################################## 
        rospy.loginfo("-D- rmotor received %0.3f" % msg.data)
        self.latest_rmotor = msg.data
        self.time_rmotor = rospy.Time.now()
        self.rmsg_count += 1
            
    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")
        
    
    ############################################################################## 
    def test_x1(self):
    ############################################################################## 
        self.latest_lmotor = 1000
        self.latest_rmotor = 2000
        self.sendmsgs([1],[0],100)
        rospy.sleep(MSG_DELAY)
        self.assertEquals(self.latest_lmotor, 1, "test_x1: lmotor value != 1 (%0.3f != 1)" % (self.latest_lmotor))
        self.assertEquals(self.latest_rmotor, 1, "test_x1: rmotor value != 1 (%0.3f != 1)" % (self.latest_rmotor))
        self.assertEquals(self.lmsg_count, 2 , "test_x1: lmsg_count != 2 (%d != 2)" %self.lmsg_count)
        self.assertEquals(self.rmsg_count, 2 , "test_x1: rmsg_count != 2 (%d != 2)" %self.rmsg_count)
        
    ############################################################################## 
    def test_x2(self):
    ############################################################################## 
        self.latest_lmotor = 1000
        self.latest_rmotor = 2000
        self.sendmsgs([2],[0],100)
        rospy.sleep(MSG_DELAY)
        self.assertEquals(self.latest_lmotor, 2, "test_x2: lmotor value != 2 (%0.3f != 2)" % (self.latest_lmotor))
        self.assertEquals(self.latest_rmotor, 2, "test_x2: rmotor value != 2 (%0.3f != 2)" % (self.latest_rmotor))
        self.assertEquals(self.lmsg_count, 2 , "test_x1: lmsg_count != 2 (%d != 2)" %self.lmsg_count)
        self.assertEquals(self.rmsg_count, 2 , "test_x1: rmsg_count != 2 (%d != 2)" %self.rmsg_count)
        
    ############################################################################## 
    def test_x1x4(self):
    ############################################################################## 
        self.latest_lmotor = 1000
        self.latest_rmotor = 2000
        self.sendmsgs([1,1,1,1],[0,0,0,0],100)
        rospy.sleep(MSG_DELAY)
        self.assertEquals(self.latest_lmotor, 1, "test_x1: lmotor value != 1 (%0.3f != 1)" % (self.latest_lmotor))
        self.assertEquals(self.latest_rmotor, 1, "test_x1: rmotor value != 1 (%0.3f != 1)" % (self.latest_rmotor))
        self.assertEquals(self.lmsg_count, 5 , "test_x1: lmsg_count != 5 (%d != 5)" %self.lmsg_count)
        self.assertEquals(self.rmsg_count, 5 , "test_x1: rmsg_count != 5 (%d != 5)" %self.rmsg_count)
        
    ############################################################################## 
    def test_xm1(self):
    ############################################################################## 
        self.latest_lmotor = 1000
        self.latest_rmotor = 2000
        self.sendmsgs([-1],[0],100)
        rospy.sleep(MSG_DELAY)
        self.assertEquals(self.latest_lmotor, -1, "test_x2: lmotor value != 2 (%0.3f != 2)" % (self.latest_lmotor))
        self.assertEquals(self.latest_rmotor, -1, "test_x2: rmotor value != 2 (%0.3f != 2)" % (self.latest_rmotor))
        self.assertEquals(self.lmsg_count, 2 , "test_x1: lmsg_count != 2 (%d != 2)" %self.lmsg_count)
        self.assertEquals(self.rmsg_count, 2 , "test_x1: rmsg_count != 2 (%d != 2)" %self.rmsg_count)
        
    ############################################################################## 
    def test_r1(self):
    ############################################################################## 
        self.latest_lmotor = 1000
        self.latest_rmotor = 2000
        self.sendmsgs([0],[1],100)
        rospy.sleep(MSG_DELAY)
        self.assertAlmostEqual(self.latest_lmotor, -(self.base_width / 2),2, "test_x2: lmotor value != 2 (%0.3f != %0.3f)" % (self.latest_lmotor, -(self.base_width / 2)))
        self.assertAlmostEqual(self.latest_rmotor, (self.base_width / 2), 2, "test_x2: rmotor value != 2 (%0.3f != %0.3f)" % (self.latest_rmotor, (self.base_width / 2))) 
        self.assertEquals(self.lmsg_count, 2 , "test_x1: lmsg_count != 2 (%d != 2)" %self.lmsg_count)
        self.assertEquals(self.rmsg_count, 2 , "test_x1: rmsg_count != 2 (%d != 2)" %self.rmsg_count)
        
    
        
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    import rostest
    rospy.loginfo("-I- test_twist_to_motors started")
    rospy.set_param("test_rate",100)
    rospy.loginfo("-D- sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, 'test_twist_to_motors', TestTwistToMotors, sys.argv) 
    