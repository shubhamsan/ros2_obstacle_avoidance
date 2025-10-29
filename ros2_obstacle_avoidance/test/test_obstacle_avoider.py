#!/usr/bin/env python3
#Unit tests for TurtleBot3 Obstacle Avoider
import unittest
import rclpy
from sensor_msgs.msg import LaserScan
from ros2_obstacle_avoidance.obstacle_avoidance_node import ObstacleAvoider
import math


class TestObstacleAvoider(unittest.TestCase):
    #Unit tests for ObstacleAvoider node
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = ObstacleAvoider()
    
    def tearDown(self):
        self.node.destroy_node()
    
    def create_turtlebot3_scan(self, front_distance=3.0, 
                               left_distance=3.0, right_distance=3.0):
        # Create realistic TurtleBot3 LaserScan message
        scan = LaserScan()
        scan.header.frame_id = 'base_scan'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.range_min = 0.12
        scan.range_max = 3.5
        
        # Create 360 readings
        ranges = [3.5] * 360
        
        # Set front readings (around index 0 and 359)
        for i in list(range(0, 20)) + list(range(340, 360)):
            ranges[i] = front_distance
        
        # Set left readings (around index 90)
        for i in range(80, 100):
            ranges[i] = left_distance
        
        # Set right readings (around index 270)
        for i in range(260, 280):
            ranges[i] = right_distance
        
        scan.ranges = ranges
        scan.intensities = [0.0] * 360
        
        return scan
    
    def test_obstacle_detected_close(self):
        # Test obstacle detection when object is close
        scan = self.create_turtlebot3_scan(front_distance=0.3)
        self.node.scan_callback(scan)
        
        self.assertTrue(self.node.obstacle_detected)
        self.assertEqual(self.node.current_state, 'AVOIDING_OBSTACLE')
        self.assertLess(self.node.min_front_distance, self.node.safety_distance)
    
    def test_no_obstacle_far(self):
        #Test no obstacle detection when path is clear.
        scan = self.create_turtlebot3_scan(front_distance=2.0)
        self.node.scan_callback(scan)
        
        self.assertFalse(self.node.obstacle_detected)
        self.assertEqual(self.node.current_state, 'MOVING_FORWARD')
        self.assertGreater(self.node.min_front_distance, self.node.safety_distance)
    
    def test_turn_direction_left_preferred(self):
        # Test that robot turns left when left side has more space.
        scan = self.create_turtlebot3_scan(
            front_distance=0.3,
            left_distance=2.0,
            right_distance=0.5
        )
        self.node.scan_callback(scan)
        
        self.assertTrue(self.node.obstacle_detected)
        self.assertGreater(self.node.min_left_distance, 
                          self.node.min_right_distance)
    
    def test_turn_direction_right_preferred(self):
        # Test that robot turns right when right side has more space.
        scan = self.create_turtlebot3_scan(
            front_distance=0.3,
            left_distance=0.5,
            right_distance=2.0
        )
        self.node.scan_callback(scan)
        
        self.assertTrue(self.node.obstacle_detected)
        self.assertGreater(self.node.min_right_distance, 
                          self.node.min_left_distance)
    
    def test_safety_distance_threshold(self):
        # Just below safety distance
        scan = self.create_turtlebot3_scan(front_distance=0.45)
        self.node.scan_callback(scan)
        self.assertTrue(self.node.obstacle_detected)
        
        # Just above safety distance
        scan = self.create_turtlebot3_scan(front_distance=0.53)
        self.node.scan_callback(scan)
        self.assertFalse(self.node.obstacle_detected)


if __name__ == '__main__':
    unittest.main()