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
        #super(TestRangeFilterCommon, self).__init__(*args)
        super(TestRangeFilterCommon, self).__init__(*args)
        
    ############################################################################## 
    def setUp(self):
    ############################################################################## 
        rospy.init_node("test_range_filter")
        self.coefficient = rospy.get_param("range_filter/coefficient", 266)
        self.exponent = rospy.get_param("range_filter/exponent", -1.31)
        self.rolling_pts = rospy.get_param("range_filter/rolling_pts",4)
        self.test_rate = rospy.get_param("~test_rate",100)
        self.latest_filtered = 1e10
        self.latest_std = 2e10
        self.range_filter_sub = rospy.Subscriber("range_filtered", Float32, self.range_filtered_callback)
        rospy.Subscriber("range_std", Float32, self.std_callback)
        self.range_pub = rospy.Publisher("range", Int16)
        self.range_pub.publish(2)
        rospy.loginfo("-D- before wait range_filter_sub publishers: %d" % self.range_filter_sub.get_num_connections())
        rospy.sleep(2)
        rospy.loginfo("-D- after wait range_filter_sub publishers: %d" % self.range_filter_sub.get_num_connections())
            
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
class TestRangeFilterValid(TestRangeFilterCommon):
############################################################################## 
############################################################################## 

    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")
    
        
    ############################################################################## 
    def test_filtered_ones(self):
    ############################################################################## 
        
        self.latest_filtered = 1e10
        self.latest_std = 2e10
        msgs = [1] * (self.rolling_pts)
        rospy.loginfo("-D- test_filtered_ones")
        self.sendmsgs(msgs, self.test_rate)
        rospy.sleep(1)
        self.assertEquals(self.latest_filtered, self.coefficient, "filtered_ones != expected (%0.3f != %0.3f)" % (self.latest_filtered, self.coefficient))
        self.assertEquals(self.latest_std,0,"filtered_ones: std != 0 (%0.3f != 0)" % self.latest_std)
        
    ############################################################################## 
    def const_value_test(self,val,testname):
    ############################################################################## 
        
        self.latest_filtered = 1e10
        self.latest_std = 2e10
        msgs = [val] * (self.rolling_pts)
        rospy.loginfo("-D- test_filtered_ones")
        self.sendmsgs(msgs, self.test_rate)
        rospy.sleep(1)
        expected = self.coefficient * val ** self.exponent
        self.assertAlmostEqual(self.latest_filtered, expected, 4,"%s: filtered_ones != expected (%0.3f != %0.3f)" % (testname, self.latest_filtered, expected))
        self.assertEquals(self.latest_std,0,"%s; filtered_ones: std != 0 (%0.3f != 0)" % (testname, self.latest_std))
        
    
    ############################################################################## 
    def test_filtered_ones(self):
    ############################################################################## 
        self.const_value_test(1, "constant_ones")
        self.const_value_test(2, "constant_twos")
        self.const_value_test(10, "constant_tens")
        self.const_value_test(100, "constant_hundreds")
        self.const_value_test(572, "constant_572")
        
    ############################################################################## 
    def test_invalid_zeros(self):
    ############################################################################## 
        
        msgs = [0] * (self.rolling_pts)
        self.latest_filtered = 1e10
        self.latest_std = 2e10
        self.sendmsgs(msgs, self.test_rate)
        rospy.sleep(1)
        self.assertEquals(self.latest_filtered,1e10,"filtered_zeros: value != 1e10 (%0.3f != 0)" % self.latest_std)
        self.assertEquals(self.latest_std,2e10,"filtered_zeros: std != 2e10 (%0.3f != 0)" % self.latest_std)
        
    ############################################################################## 
    def test_invalid_neg(self):
    ############################################################################## 
        
        msgs = [-1] * (self.rolling_pts)
        # rospy.loginfo("-D- test_invalid_neg")
        self.sendmsgs(msgs, self.test_rate)
        self.latest_filtered = 1e10
        self.latest_std = 2e10
        rospy.sleep(1)
        self.assertEquals(self.latest_filtered,1e10,"filtered_neg: std != 0 (%0.3f != 1e10)" % self.latest_std)
        self.assertEquals(self.latest_std,2e10,"filtered_neg: std != 0 (%0.3f != 2e10)" % self.latest_std)
#        self.assertEquals(self.latest_std,0,"filtered_neg: std != 0 (%0.3f != 2e10)" % self.latest_std)
        
        
        
############################################################################## 
############################################################################## 
#class TestRangeFilterMinValidNeg(TestRangeFilter):
############################################################################## 
############################################################################## 

    ############################################################################## 
#    def test_one_equals_one(self):
    ############################################################################## 
#        rospy.loginfo("-D- test_one_equals_one")
#        self.assertEquals(1, 1, "1!=1")

 
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    import rostest
    rospy.loginfo("-I- test_range_filter started")
    rospy.set_param("test_rate",100)
    rospy.loginfo("-D- sys.argv: %s" % str(sys.argv))
    rostest.rosrun(PKG, 'test_range_filter_valid', TestRangeFilterValid, sys.argv) 
    
#    rospy.set_param("range_filter/coefficient", 266)
#    rostest.rosrun(PKG, 'test_range_filter_valid_c266', TestRangeFilterValid, sys.argv) 
#    rostest.rosrun(PKG, 'test_range_filter_invalid', TestRangeFilterInvalid, sys.argv)
    
#    rostest.rosrun(PKG, 'test_range_filter_minvalidneg', TestRangeFilterMinValidNeg, sys.argv)
