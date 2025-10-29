#!/usr/bin/env python3
#importing the necessary library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from datetime import datetime
import os


class StatusLogger(Node):
    #Logs robot status including state, distance, and velocities
    
    def __init__(self):
        super().__init__('status_logger')
        
        #Declare  the robots parameters
        self.declare_parameter('log_to_file', True)
        self.declare_parameter('log_file', 'turtlebot3_status.log')
        self.declare_parameter('log_directory', os.path.expanduser('~/ros2_logs'))
        
        #Get parameters
        self.log_to_file = self.get_parameter('log_to_file').value
        log_file = self.get_parameter('log_file').value
        log_dir = self.get_parameter('log_directory').value
        
        # Create log directory
        if self.log_to_file:
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(log_dir, log_file)
            
            self.log_file_handle = open(log_path, 'w')
            self.log_file_handle.write(
                'Timestamp,State,Front_Dist,Left_Dist,Right_Dist,'
                'Linear_Vel,Angular_Vel\n'
            )
            self.get_logger().info(f'Logging to: {log_path}')
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            String, 'robot_status', self.status_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # State variables
        self.current_cmd_vel = Twist()
        self.log_count = 0
        
        self.get_logger().info('Status Logger initialized')
    
    def status_callback(self, msg: String):
        # Log robot status
        # Parse status message: STATE|FRONT|LEFT|RIGHT
        parts = msg.data.split('|')
        if len(parts) == 4:
            state, front_dist, left_dist, right_dist = parts
            
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # Console log
            log_entry = (
                f'{timestamp} | {state:20s} | '
                f'F:{front_dist:>5s}m L:{left_dist:>5s}m R:{right_dist:>5s}m | '
                f'LinVel:{self.current_cmd_vel.linear.x:>5.2f} '
                f'AngVel:{self.current_cmd_vel.angular.z:>5.2f}'
            )
            
            self.get_logger().info(log_entry, throttle_duration_sec=1.0)
            
            # To log the File 
            if self.log_to_file:
                file_entry = (
                    f'{timestamp},{state},{front_dist},{left_dist},{right_dist},'
                    f'{self.current_cmd_vel.linear.x:.3f},'
                    f'{self.current_cmd_vel.angular.z:.3f}\n'
                )
                self.log_file_handle.write(file_entry)
                self.log_file_handle.flush()
                
                self.log_count += 1
                if self.log_count % 50 == 0:
                    self.get_logger().info(
                        f'Logged {self.log_count} entries')
    
    def cmd_vel_callback(self, msg: Twist):
        # Store current velocity commands
        self.current_cmd_vel = msg
    
    def __del__(self):
        #Close log file on destruction
        if hasattr(self, 'log_file_handle'):
            self.log_file_handle.close()
            self.get_logger().info(f'Total entries logged: {self.log_count}')


def main(args=None):
    rclpy.init(args=args)
    node = StatusLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down logger node...')
    finally:
        node.destroy_node() 
        rclpy.shutdown()


if __name__ == '__main__':
    main()