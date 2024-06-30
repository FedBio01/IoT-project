import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

from geometry_msgs.msg import Twist, Point

from sim_utils import EventScheduler

import math
import sys

from nav_msgs.msg import Odometry

import math_utils

SIMULATION_SIDE = int(sys.argv[1])

WORLD_NAME = "iot_project_world"

class SensorController(Node):

    def __init__(self):
        super().__init__('sensor_controller')

        self.id = self.declare_parameter('id', -1)

        self.tx_topic = self.create_publisher(
            String,
            f'/ActiveSensor_{self.id.get_parameter_value().integer_value}/tx_data',
            10
        )

        self.generated_data = 0
        self.position = Point(x = 0.0, y = 0.0, z = 0.0)

        self.yaw = 0  # Current yaw angle of the robot

        # control sensor direction movement
        self.move_backward = False
        self.move_forward = True
        

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        self.odometry_subscrber = self.create_subscription(
            Odometry,
            f'/ActiveSensor_{self.id.get_parameter_value().integer_value}/odometry',
            self.store_position,
            10
        )

        
        self.vel_pub = self.create_publisher(
            Twist,
            f'/ActiveSensor_{self.id.get_parameter_value().integer_value}/cmd_vel',
            10
        )
        
        self.event_scheduler.schedule_event(1, self.publish_velocity)

        
        self.event_scheduler.schedule_event(1, self.simple_publish) # comment this line if you want to use the version below

        ### Version for different sensors generating data at different frequencies
        ### Note: to use this version be aware of changing also "base_station_controller" code in accordance with this
        # self.event_scheduler.schedule_event(1+self.id.get_parameter_value().integer_value, self.simple_publish)
        ###

    def simple_publish(self):

        id = self.id.get_parameter_value().integer_value

        msg = String()
        msg.data = f"{id}_{self.generate_data()}"

        self.tx_topic.publish(msg) 


    def generate_data(self):

        self.generated_data += 1
        return self.generated_data


    def store_position(self, odometry_msg : Odometry):

        self.position.x = odometry_msg.pose.pose.position.x
        self.position.y = odometry_msg.pose.pose.position.y
        self.position.z = odometry_msg.pose.pose.position.z

        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )


    # control sensor movement. 
    # for semplicity, we make them move along their X-axis, 
    # after spawning on a random point of the area (in the sdf_launch.py file)
    def publish_velocity(self):
        twist_msg = Twist()
        self.store_position
    
        if self.move_forward:
            twist_msg.linear.x = 0.5

        elif self.move_backward:
            twist_msg.linear.x = -0.5
        
        # when the sensor is reaching the border of the area, it comes back 
        if self.position.x >= SIMULATION_SIDE-3:
            self.move_forward = False
            self.move_backward = True

        elif self.position.x <= -SIMULATION_SIDE+3:
            self.move_forward = True

        self.vel_pub.publish(twist_msg)






def main():
    
    rclpy.init()
    executor = MultiThreadedExecutor()

    sensor_controller = SensorController()
    executor.add_node(sensor_controller)

    executor.spin()

    sensor_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
