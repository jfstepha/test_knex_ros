#!/usr/bin/env python
PKG = 'unit_test_test'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

## A sample python unit test
class TestBareBones(unittest.TestCase):
    
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")
        
    def test_two_equals_two(self):
        self.assertEquals(2,2, "2!=2")
        
    def test_two_ne_one(self):
        self.assertNotEqual(2, 1, "2==1")
        
#    def test_two_equals_one(self):
#        self.assertEqual(2, 1, "2==1")
        
 
if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestBareBones) 
