#!/usr/bin/env python
PKG = 'test_knex_ros'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
sys.path.append('/home/jfstepha/ros_workspace/knex_ros/nodes')


import unittest
import range_filter
from std_msgs.msg import Float32
from std_msgs.msg import Int16


############################################################################## 
class TestCommon(unittest.TestCase):
############################################################################## 
    ############################################################################## 
    def setUp(self):
    ############################################################################## 
        self.test_rate = rospy.get_param("~test_rate",100) 
        self.myRangeFilter = range_filter.RangeFilter()
        self.rolling_pts =self.myRangeFilter.rolling_pts
    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        self.assertEquals(1, 1, "1!=1")
        
        
    ############################################################################## 
    def sendmsgs(self, msgs, rate):
    ############################################################################## 
        r = rospy.Rate(rate)
        # rospy.loginfo("-D- sendmsgs: sending %s" % str(msgs))
        my_range = Int16()
        for m in msgs:
            my_range.data = m     
            self.myRangeFilter.inputCallback(my_range)
            r.sleep()
            
############################################################################## 
class TestValid(TestCommon):
############################################################################## 
    ############################################################################## 
    def test_ones(self):
    ############################################################################## 
    # make sure we can send a bunch of ones and get the coefficient back
        msgs = [1] * (self.rolling_pts)
        self.sendmsgs(msgs, self.test_rate)
        self.assertEquals(self.myRangeFilter.rolling_meters, self.myRangeFilter.b, "test_ones: rolling_meters != b (%0.3f != %0.3f" %(self.myRangeFilter.rolling_meters, self.myRangeFilter.b))
        
    ############################################################################## 
    def test_coeff(self):
    ############################################################################## 
    # make sure we can get the coefficient back no matter what that may be
        rospy.loginfo("-D- test_coeff")
        self.do_coeff_test(1,"ones")
        self.do_coeff_test(2,"twos")
        self.do_coeff_test(5,"five")
        self.do_coeff_test(10,"ten")
        self.do_coeff_test(20,"twenty")
        self.do_coeff_test(100,"onehundred")
        self.do_coeff_test(1000,"onethousand")
        self.do_coeff_test(10000,"tenthousand")
        self.do_coeff_test(100000,"onehundredthousand")
        
    ############################################################################## 
    def do_coeff_test(self, val, msg):
    ############################################################################## 
    # actually does the work for the test above
        msgs = [1] * (self.rolling_pts)
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.b = val 
        self.sendmsgs(msgs, self.test_rate)
        self.assertEquals(self.myRangeFilter.rolling_meters, val, "test_coeff %s: rolling_meters != b (%0.3f != %0.3f)" %(msg, self.myRangeFilter.rolling_meters, val))
        self.assertEquals(self.myRangeFilter.rolling_std,0,"test_coeff %s: std_dev != 0 (%0.3f != 0.0)" % (msg, self.myRangeFilter.rolling_std))
        
    ############################################################################## 
    def test_num_pts_flush(self):
    ############################################################################## 
    # test to make sure we can send one more than num_pts, and it will get flushed out
        self.do_test_num_pts_flush(4)
        self.do_test_num_pts_flush(5)
        self.do_test_num_pts_flush(10)
        self.do_test_num_pts_flush(100)
        self.do_test_num_pts_flush(1000)
        self.rolling_pts = 4
        self.myRangeFilter.rolling_pts = 4
        
    
    ############################################################################## 
    def do_test_num_pts_flush(self, val):
    ############################################################################## 
        self.rolling_pts = val
        self.myRangeFilter.rolling_pts = val 
        msgs = [10] + [1] * (self.rolling_pts) 
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.b = 10
        self.sendmsgs(msgs, self.test_rate)
        self.assertEquals(self.myRangeFilter.rolling_meters, 10,"test_num_pts %d pts: rolling_meters != b (%0.3f != 10.0)" %(val, self.myRangeFilter.rolling_meters))
        self.assertEquals(self.myRangeFilter.rolling_std,0,"test_num_pts %d: std_dev != 0 (%0.3f != 0.0)" % (val, self.myRangeFilter.rolling_std))
        
    ############################################################################## 
    def test_std_1(self):
    ############################################################################## 
        msgs = [1,1,1,2]
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.rolling_std = 2000
        self.sendmsgs(msgs, self.test_rate)
        
        self.assertAlmostEqual(self.myRangeFilter.rolling_meters, 198.577, 3,"test_std_1 average != 1.25 (%0.3f != 198.577)"  % self.myRangeFilter.rolling_meters)
        self.assertAlmostEqual(self.myRangeFilter.rolling_std, 0.433, 3, "test_td_1 stdev != 0.5 (%0.3f != 0.5" % self.myRangeFilter.rolling_std)
        
    ############################################################################## 
    def test_std_2(self):
    ############################################################################## 
        self.rolling_pts = 10
        self.myRangeFilter.rolling_pts = 10
        msgs = [1,2,3,4,5,6,7,8,9,10]
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.rolling_std = 2000
        self.sendmsgs(msgs, self.test_rate)
        
        self.assertAlmostEqual(self.myRangeFilter.rolling_meters, 16.119,3,"test_std_2 average != 16.119 (%0.4f != 1.25)" % self.myRangeFilter.rolling_meters)
        self.assertAlmostEqual(self.myRangeFilter.rolling_std, 1.118,3, "test_td_2 stdev != 0.5 (%0.3f != 0.5)" % self.myRangeFilter.rolling_std)
        
        self.rolling_pts = 4
        self.myRangeFilter.rolling_pts = 4
        
    ############################################################################## 
    def test_exp_1(self): self.do_exp_test(1, 2, 3 , "1_2_3")
    def test_exp_2(self): self.do_exp_test(2, 3, 4 , "2_3_4")
    def test_exp_3(self): self.do_exp_test(10, 20, 30 , "10_20_30")
    def test_exp_4(self): self.do_exp_test(10, 2, 100 , "10_20_30")
    
    ############################################################################## 
    def do_exp_test(self, m, b, val, msg):
    ############################################################################## 
        msgs = [val] * (self.rolling_pts) 
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.b = b 
        self.myRangeFilter.m = m
        self.sendmsgs(msgs, self.test_rate)
        expected = b * val ** m
        self.assertAlmostEquals(self.myRangeFilter.rolling_meters, expected,"test_exp %s: rolling_meters != expected (%0.3f != %0.3f)" 
                                % (msg, self.myRangeFilter.rolling_meters, expected))
        self.assertEquals(self.myRangeFilter.rolling_std,0,"test_num_pts %d: std_dev != 0 (%0.3f != 0.0)" 
                          % (val, self.myRangeFilter.rolling_std))
        
        
    ############################################################################## 
    def test_invalid_zeros(self):
    ############################################################################## 
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.rolling_std = 2000
        msgs = [0] * self.rolling_pts
        
        self.sendmsgs(msgs, self.test_rate)
        self.assertEqual(self.myRangeFilter.rolling_meters, 1000, "test_invalid_zeros")
        self.assertEqual(self.myRangeFilter.rolling_std, 2000, "test_invalid_zeros")
        
    ############################################################################## 
    def test_invalid_neg(self):
    ############################################################################## 
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.rolling_std = 2000
        msgs = [-1,-2,-3,-4] * self.rolling_pts
        
        self.sendmsgs(msgs, self.test_rate)
        self.assertEqual(self.myRangeFilter.rolling_meters, 1000, "test_invalid_neg")
        self.assertEqual(self.myRangeFilter.rolling_std, 2000, "test_invalid_neg")
        
    ############################################################################## 
    def test_one_invalid(self):
    ############################################################################## 
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.rolling_std = 2000
        msgs = [1,2,3,4,5,-1,6,7,8,9,10]
        self.myRangeFilter.rolling_meters = 1000
        self.myRangeFilter.rolling_std = 2000
        self.sendmsgs(msgs, self.test_rate)
        
        self.assertAlmostEqual(self.myRangeFilter.rolling_meters, 16.119,3,"test_one_invalid mean != 16.119 (%0.4f != 1.25)" % self.myRangeFilter.rolling_meters)
        self.assertAlmostEqual(self.myRangeFilter.rolling_std, 1.118,3, "test_one_invalid std_dev != 0.5 (%0.3f != 0.5)" % self.myRangeFilter.rolling_std)
        
        self.rolling_pts = 4
        self.myRangeFilter.rolling_pts = 4
    
        
        
        
if __name__ == '__main__':    
    import rostest
    rospy.loginfo("-I- noros_range_filter started")
    rostest.rosrun(PKG, 'test_range_filter_valid', TestValid, sys.argv) 
#    unittest.main()
    