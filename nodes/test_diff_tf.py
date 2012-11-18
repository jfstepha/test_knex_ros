#!/usr/bin/env python
PKG = 'test_knex_ros'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest
from optparse import OptionParser
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

import rospy

############################################################################## 
class TestDiffTfSimple(unittest.TestCase):
############################################################################## 
    ############################################################################## 
    def setUp(self):
    ############################################################################## 
        rospy.loginfo("-D- simple test")
        rospy.init_node("test_diff_tf")
        self.ticks_meter = float(rospy.get_param('ticks_meter', 50))
        self.base_width = float(rospy.get_param('~base_width', 0.245))
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        
        self.lwheel_pub = rospy.Publisher("lwheel", Int16)
        self.rwheel_pub = rospy.Publisher("rwheel", Int16)
        
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.latestOdom = Odometry()
        
    ############################################################################## 
    def odomCallback(self, msg):
    ############################################################################## 
        self.latestOdom = msg
        

############################################################################## 
class TestDiffTfValid(TestDiffTfSimple):
############################################################################## 
    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")
        
    ############################################################################## 
    def test_no_data(self):
    ############################################################################## 
        rospy.loginfo("-D- test_no_data")
        blankOdom = Odometry()
        self.assertEquals(blankOdom, self.latestOdom,"blank odom is not blank (%s != %s)" %(str(blankOdom), str(self.latestOdom)))
        rospy.loginfo("-I- test_no_data_done")
        
    ############################################################################## 
    def test_drive_1m(self):
    ############################################################################## 
        rospy.loginfo("-I- starting test_drive_1m")
        rospy.sleep(.1)
        self.lwheel_pub.publish(self.ticks_meter)
        self.rwheel_pub.publish(self.ticks_meter)
        rospy.sleep(.1)
        self.assertEqual(self.latestOdom.pose.pose.position.x, 1.0, "odomx != 1 (%0.3f)" % self.latestOdom.pose.pose.position.x)
        rospy.loginfo("-I- finished test_drive_1m")
        
        
        
############################################################################## 
class TestDiffTfInValid(TestDiffTfSimple):
############################################################################## 
    ############################################################################## 
    def test_one_equals_one_invalid(self):
    ############################################################################## 
        rospy.loginfo("-D- test_one_equals_one_invalid")
        self.assertEquals(1, 1, "1!=1")
        
    
        
    
############################################################################## 
if __name__ == '__main__':
############################################################################## 
    import rostest
    rospy.loginfo("-I- test_diff_tf started")
    rostest.rosrun(PKG, 'test_range_filter_valid', TestDiffTfValid, sys.argv) 
    # rostest.rosrun(PKG, 'test_range_filter_invalid', TestDiffTfInValid, sys.argv) 
    
