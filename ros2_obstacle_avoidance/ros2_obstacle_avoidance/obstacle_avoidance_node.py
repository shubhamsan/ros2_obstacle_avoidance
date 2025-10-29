#!/usr/bin/env 
#Importing the Library 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
import math
import numpy as np


class ObstacleAvoider(Node):
    #################Main navigation node with obstacle avoidance for TurtleBot3######################
    
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Declare parameters (optimized for TurtleBot3)
        self.declare_parameter('safety_distance', 0.5)  # TurtleBot3 range is 0.12-3.5m
        self.declare_parameter('linear_speed', 0.22)    # TurtleBot3 max: ~0.26 m/s
        self.declare_parameter('angular_speed', 1.5)    # TurtleBot3 max: ~1.82 rad/s
        self.declare_parameter('front_angle_range', 35.0)  # degrees to check in front
        self.declare_parameter('side_angle_range', 30.0)   # degrees to check on sides
        
        # Get parameters from params file or else default
        self.safety_distance = self.get_parameter('safety_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.front_angle_range = math.radians(
            self.get_parameter('front_angle_range').value
        )
        self.side_angle_range = math.radians(
            self.get_parameter('side_angle_range').value
        )

        self.add_on_set_parameters_callback(self.parameter_callback)
        
        
        #Create a Publisher & subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        
        #State variables
        self.current_state = 'INITIALIZING'
        self.obstacle_detected = False
        self.min_front_distance = float('inf')
        self.min_left_distance = float('inf')
        self.min_right_distance = float('inf')
        self.scan_received = False
        
        self.get_logger().info('#' * 100)
        self.get_logger().info('TurtleBot3 Obstacle Avoider initialized')
        self.get_logger().info(f'Safety distance: {self.safety_distance}m')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info('#' * 100)
        
        #For Low pass filter
        self.filtered_angular_z=0
        self.last_turn_direction = ""

    def parameter_callback(self, params):
        #Callback executed when parameters are set via ros2 param set
        for param in params:

            if param.name in ['safety_distance', 'linear_speed', 'angular_speed', 'front_angle_range', 'side_angle_range']:
                # Update the internal variable with the new value
                setattr(self, param.name, param.value)
                self.get_logger().info(f'Parameter updated: {param.name} set to {self.angular_speed}')
        
        # Must return a SetParametersResult message
        return SetParametersResult(successful=True)
    
    def scan_callback(self, msg: LaserScan):
        #Process LIDAR data "
        self.scan_received = True
        
        # TurtleBot3 scan: angle_min=0, angle_max=2π, in counter-clockwise direction
        num_readings = len(msg.ranges)
        
        # Convert angles to indices
        # Front: 0° (index 0 and index -1)
        # Left: 90° (index ~num_readings/4) as its counter clockwise
        # Right: 270° (index ~3*num_readings/4)
        
        # Calculate sectors
        front_indices = self.get_sector_indices(
            num_readings, 0, self.front_angle_range
        )
        #self.get_logger().info(f'Front indices: {front_indices}')
        left_indices = self.get_sector_indices(
            num_readings, math.pi/2, self.side_angle_range
        )
        #self.get_logger().info(f'left indices: {left_indices}')

        right_indices = self.get_sector_indices(
            num_readings, -math.pi/2, self.side_angle_range
        )
        #self.get_logger().info(f'right indices: {right_indices}')

        
        # Get minimum distances in each direction
        self.min_front_distance = self.get_min_distance(msg, front_indices)
        self.min_left_distance = self.get_min_distance(msg, left_indices)
        self.min_right_distance = self.get_min_distance(msg, right_indices)
        
        # Check for obstacles
        self.obstacle_detected = self.min_front_distance < self.safety_distance
        
        # Execute navigation logic
        self.navigate()
    
    def get_sector_indices(self, num_readings: int, center_angle: float, 
                           range_angle: float) -> list:
        #Get indices for a sector of the scan
        # Normalize center_angle to [0, 2π] range 
        center_angle = center_angle % (2 * math.pi)
        
        # Calculate start and end angles
        start_angle = (center_angle - range_angle/2) % (2 * math.pi)
        end_angle = (center_angle + range_angle/2) % (2 * math.pi)
        #self.get_logger().info(f'start_angle: {start_angle}, end_angle: {end_angle}')
        
        # Convert to indices
        start_idx = int(start_angle / (2 * math.pi) * num_readings)
        end_idx = int(end_angle / (2 * math.pi) * num_readings)
        
        #Handle wraparound
        if start_idx <= end_idx:
            return list(range(start_idx, end_idx))
        else:
            return list(range(start_idx, num_readings)) + list(range(0, end_idx))
    
    def get_min_distance(self, msg: LaserScan, indices: list) -> float:
        #Get minimum valid distance from scan indices 
        valid_ranges = []
        for i in indices:
            r = msg.ranges[i]
            if not math.isinf(r) and not math.isnan(r) and msg.range_min < r < msg.range_max:
                valid_ranges.append(r)
        
        return min(valid_ranges) if valid_ranges else msg.range_max
    
    def navigate(self):
        #"Main navigation logic with intelligent turning
        cmd = Twist()
        
        if not self.scan_received:
            self.current_state = 'WAITING_FOR_SCAN'
            self.get_logger().warn('Waiting for lidar data...')
            return
        
        if self.obstacle_detected:
            #Obstacle in front - decide turn direction
            self.current_state = 'AVOIDING_OBSTACLE'
            
            # Turn towards the side with more space
            # if self.min_left_distance > self.min_right_distance:
            #     turn_direction = 'LEFT'
            #     raw_angular_z = self.angular_speed
            # else:
            #     turn_direction = 'right'
            #     raw_angular_z = -self.angular_speed
            
            DIFFERENCE_THRESHOLD = 0.1 

            if self.min_left_distance > self.min_right_distance + DIFFERENCE_THRESHOLD:
                turn_direction = 'LEFT'
                self.last_turn_direction = turn_direction

                raw_angular_z = self.angular_speed
            elif self.min_right_distance > self.min_left_distance + DIFFERENCE_THRESHOLD:
                turn_direction = 'RIGHT'
                self.last_turn_direction = turn_direction
                raw_angular_z = -self.angular_speed
            else:
                # Distances are too close, maintain previous turn or default to a safe side  left
                # This prevents rapid switching (oscillation)
                if self.last_turn_direction == 'RIGHT':
                    turn_direction = 'RIGHT'
                    raw_angular_z = -self.angular_speed
                else: # Default to left
                    turn_direction = 'LEFT'
                    raw_angular_z = self.angular_speed
            
            cmd.linear.x = 0.0
            cmd.angular.z = raw_angular_z
            self.get_logger().info(
                f'AVOIDING: Front={self.min_front_distance:.2f}m | '
                f'Turning {turn_direction} | '
                f'L={self.min_left_distance:.2f}m R={self.min_right_distance:.2f}m')
        else:
            # just to move forward
            self.current_state = 'MOVING_FORWARD'
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            
            self.get_logger().info(
                f'Forward: Front={self.min_front_distance:.2f}m | '
                f'L={self.min_left_distance:.2f}m R={self.min_right_distance:.2f}m')
        
        # Publish commands
        self.cmd_vel_pub.publish(cmd)
        
        #Publish status for logger
        status_msg = String()
        status_msg.data = (
            f'{self.current_state}|{self.min_front_distance:.2f}|'
            f'{self.min_left_distance:.2f}|{self.min_right_distance:.2f}'
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down....')
    finally:
        #Stop robot on shutdown
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()