#!/usr/bin/env python
PKG = 'unit_test_test'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest
import threading
from optparse import OptionParser

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

## A sample python unit test
############################################################################## 
############################################################################## 
class TestBareBones(unittest.TestCase):
############################################################################## 
############################################################################## 
    
    ## test 1 == 1
    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        self.assertEquals(1, 1, "1!=1")
        
    ############################################################################## 
    def test_two_ne_one(self):
    ############################################################################## 
        self.assertNotEqual(2, 1, "2==1")

### this one intentionally fails 
#    def test_two_equals_one(self):
#        self.assertEqual(2, 1, "2==1")

############################################################################## 
############################################################################## 
class TestRangeFilterCommon(unittest.TestCase):
############################################################################## 
############################################################################## 
    # def setUp(self):
    ############################################################################## 
    def __init__(self, *args):
    ############################################################################## 
        # rospy.loginfo("-D- TestRangeFilter __init__")
        super(TestRangeFilterCommon, self).__init__(*args)
        
        rospy.init_node("test_range_filter")
        self.coefficient = rospy.get_param("range_filter/coefficient", 266)
        self.exponent = rospy.get_param("range_filter/exponent", -1.31)
        self.rolling_pts = rospy.get_param("range_filter/rolling_pts",4)
        self.test_rate = rospy.get_param("~test_rate",100)
        self.latest_filtered = -1e99
        self.latest_std = -1e99
        self.range_filter_sub = rospy.Subscriber("range_filtered", Float32, self.range_filtered_callback)
        rospy.Subscriber("range_std", Float32, self.std_callback)
        self.range_pub = rospy.Publisher("range", Int16)
        rospy.sleep(1)
            
    ############################################################################## 
    def sendmsgs(self, msgs, rate):
    ############################################################################## 
        r = rospy.Rate(rate)
        # rospy.loginfo("-D- sendmsgs: sending %s" % str(msgs))
        for m in msgs:
            rospy.loginfo("-D- publishing %d" % m) 
            self.range_pub.publish(m)
            r.sleep()
            
        
    ############################################################################## 
    def range_filtered_callback(self, msg):
    ############################################################################## 
        rospy.loginfo("-D- range_filtered received %0.3f" % msg.data)
        self.latest_filtered = msg.data
        self.time_filtered = rospy.Time.now()
            
    ############################################################################## 
    def std_callback(self, msg):
    ############################################################################## 
        self.latest_std = msg.data
        self.time_std = rospy.Time.now()
        
############################################################################## 
############################################################################## 
class TestRangeFilter(TestRangeFilterCommon):
############################################################################## 
############################################################################## 

    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")
    
    ############################################################################## 
    def test_filtered_zeros(self):
    ############################################################################## 
        
        msgs = [0] * (self.rolling_pts)
        # rospy.loginfo("-D- test_filtered_zeros")
        self.sendmsgs(msgs, self.test_rate)
        rospy.sleep(1)
        self.assertGreater(self.latest_filtered, 1e30,"filtered_zeros != 0 (%0.3f != inf)" % (self.latest_filtered))
        self.assertEquals(self.latest_std,0,"filtered_zeros: std != 0 (%0.3f != 0)" % self.latest_std)
        
    ############################################################################## 
    def test_filtered_ones(self):
    ############################################################################## 
        
        msgs = [1] * (self.rolling_pts)
        # rospy.loginfo("-D- test_filtered_ones")
        self.sendmsgs(msgs, self.test_rate)
        rospy.sleep(1)
        self.assertEquals(self.latest_filtered, self.coefficient,"filtered_ones != coefficient (%0.3f != %0.3f)" % (self.latest_filtered, self.coefficient))
        self.assertEquals(self.latest_std,0,"filtered_ones: std != 0 (%0.3f != 0)" % self.latest_std)
        
    ############################################################################## 
    def test_filtered_neg(self):
    ############################################################################## 
        
        msgs = [-1] * (self.rolling_pts)
        # rospy.loginfo("-D- test_filtered_neg")
        self.sendmsgs(msgs, self.test_rate)
        rospy.sleep(1)
        self.assertEquals(self.latest_filtered, self.coefficient,"filtered_ones != coefficient (%0.3f != %0.3f)" % (self.latest_filtered, self.coefficient))
        self.assertEquals(self.latest_std,0,"filtered_ones: std != 0 (%0.3f != 0)" % self.latest_std)
        
        
############################################################################## 
############################################################################## 
#class TestRangeFilterMinValidNeg(TestRangeFilter):
############################################################################## 
############################################################################## 
 
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    import rostest
    rospy.loginfo("-I- test_range_filter started")
#    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones) 
    rospy.set_param("range_filter/coefficient", 10)
    rospy.set_param("test_rate",100)
    rostest.rosrun(PKG, 'test_range_filter', TestRangeFilter, sys.argv) 
