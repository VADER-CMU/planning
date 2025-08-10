#!/usr/bin/env python3

import unittest
import rospy
import rostest

class ExampleTest(unittest.TestCase):
    def test_sum(self):
        result = 1 + 1
        print("Checking that 1 + 1 equals 2")  # This line outputs to the test log
        self.assertEqual(result, 2, "Addition does not match expectation")

if __name__ == '__main__':
    rostest.rosrun('vader_planner', 'example_test', ExampleTest)
