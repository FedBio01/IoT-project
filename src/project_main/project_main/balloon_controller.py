import time
import math
import threading
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry
import dearpygui.dearpygui as dpg
from sim_utils import EventScheduler
from rosgraph_msgs.msg import Clock

import math_utils
from project_interfaces.action import Patrol


MIN_ALTITUDE_TO_PERFORM_PATROL = 15
balloon_id = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])
CACHE_SIZE = NUMBER_OF_SENSORS*10 
WORLD_NAME = "iot_project_world"

class BalloonController(Node):

    def __init__(self):
        super().__init__("drone_controller")

        self.cache = []
        self.cache_size = CACHE_SIZE

        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.yaw = 0

        self.stop_msg = Twist()

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.odometry_subscrber = self.create_subscription(
            Odometry,
            'odometry',
            self.store_position,
            10
        )

        self.rx_data = self.create_subscription(
            String,
            'rx_data',
            self.rx_callback,
            10
        )

        self.patrol_action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            self.execute_patrol_action
        )

        self.tx_data = self.create_publisher(
            String,
            'tx_data',
            10
        )

        # Subscription to the topic where the base station publishes its requests. 
        # Useful to perform LRU cache management
        self.base_station_requests = self.create_subscription(
            String,
            '/base_station/requests',
            lambda msg: self.handle_request(msg),
            NUMBER_OF_SENSORS * 3  # base station is performing 3 requests per sensor
        )

        self.event_scheduler = EventScheduler()
        self.clock_topic = self.create_subscription(
            Clock,
            f'/world/{WORLD_NAME}/clock',
            self.event_scheduler.routine,
            10
        )

        # In case a balloon doesn't receive data anymore, 
        # it will continue to order its cache based on the requests
        self.event_scheduler.schedule_event(1, self.order_cache)

        # Update the threshold with regular frequency
        self.event_scheduler.schedule_event(12, self.update_delta_t)


        self.balloon_data = [] # balloon cache
        self.request = []      # base station requests
        self.delta_t = -1      # threshold inizialization

        # create a dedicated thread to handle the GUI
        self.gui_thread = threading.Thread(target=self.init_gui)
        self.gui_thread.start()

        self.new_request_received = False # tells if it's needed to perform LRU ordering cache


    ########## LRU cache policy

    # request arrival
    def handle_request(self, msg: String):
        self.request.append(msg.data)
        self.new_request_received = True

    # If the requested data is in the cache, 
    # put it on the bottom of the list (top one will be popped further)
    def order_cache(self):
        if self.new_request_received:
            for new_request in self.request:
                if new_request in self.balloon_data:
                    index = self.balloon_data.index(new_request)
                    removed_element = self.balloon_data.pop(index)
                    self.balloon_data.append(removed_element)
            self.request = []
            self.new_request_received = False
    ##########

    ########## Delta-t threshold management

    # update threshold
    def update_delta_t(self):
        self.delta_t = self.delta_t + 1

    # remove all data with a too old timestamp
    def remove_old_ts(self):
        for element in self.balloon_data:
            current_ts = int(element.split("_")[1])
            if current_ts <= self.delta_t:
                self.balloon_data.remove(element)
    ##########

    # Sensor data arrival
    def rx_callback(self, msg: String):
        self.get_logger().info(msg.data)

        # 1. LRU ordering
        self.order_cache() 

        # 2. If needed, pop the first element of the cache
        if len(self.balloon_data) >= CACHE_SIZE:
            self.balloon_data.pop(0)

        # 3. Append the new data to the cache
        self.balloon_data.append(msg.data)

        # 4. Delete old data
        self.remove_old_ts()

        # 5. Publish the cache
        self.cache_publish()

    def cache_publish(self):
        cache_msg = String()
        cache_msg.data = " ".join(self.balloon_data)
        self.tx_data.publish(cache_msg)
        self.update_gui()
        

    def store_position(self, odometry_msg: Odometry):
        self.position = odometry_msg.pose.pose.position
        self.yaw = math_utils.get_yaw(
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        )

    def execute_patrol_action(self, goal: ServerGoalHandle):
        command_goal: Patrol.Goal = goal.request
        self.get_logger().info(f"Action requested. Performing movement to targets:\n\t{command_goal.targets}")
        self.fly_to_altitude(MIN_ALTITUDE_TO_PERFORM_PATROL)
        targets_patrolled = 0
        for target in command_goal.targets:
            self.rotate_to_target(target)
            self.move_to_target(target)
            self.get_logger().info(f"Movement to target {targets_patrolled} completed!")
            targets_patrolled += 1
        goal.succeed()
        result = Patrol.Result()
        result.result = "Movement completed"
        return result

    def fly_to_altitude(self, altitude):
        # Instantiate the move_up message
        move_up = Twist()
        move_up.linear = Vector3(x=0.0, y=0.0, z=1.0)
        move_up.angular = Vector3(x=0.0, y=0.0, z=0.0)

        self.cmd_vel_publisher.publish(move_up)

        # Wait for the drone to reach the desired altitude
        while self.position.z < altitude:
            time.sleep(0.1)

        # Stop movement after the altitude has been reached
        stop_mov = Twist()
        stop_mov.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_mov.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_mov)

    def rotate_to_target(self, target, eps=0.5):
        # We compute the angle between the current target position and the target
        # position here
        target_angle = math_utils.angle_between_points(self.position, target)
        angle_to_rotate = target_angle - self.yaw

        # Normalize the angle difference to be within the range [-pi, pi]
        angle_to_rotate = (angle_to_rotate + math.pi) % (2 * math.pi) - math.pi

        # And then assign the direction of the rotation correctly
        rotation_dir = 1 if angle_to_rotate < 0 else -1

        # Prepare the cmd_vel message
        move_msg = Twist()
        move_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        move_msg.angular = Vector3(x=0.0, y=0.0, z=0.5 * rotation_dir)
        self.cmd_vel_publisher.publish(move_msg)

        # Publish the message until the correct rotation is reached (accounting for some eps error)
        while abs(angle_to_rotate) > eps:
            angle_to_rotate = target_angle - self.yaw

        # When done, send a stop message to be sure that the drone doesn't
        # overshoot its target
        stop_msg = Twist()
        stop_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
        stop_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        self.cmd_vel_publisher.publish(stop_msg)

    def move_to_target(self, target, eps=0.5, angle_eps=0.02):
        # Save the target position and compute the distance
        distance = math_utils.point_distance(self.position, target)

        # Keep publishing movement while the distance is greater than the given EPS
        while distance > eps:
            # Compute the move vector with the given position and target
            mv = math_utils.move_vector(self.position, target)

            twist_msg = Twist()
            twist_msg.linear.x = mv[0]
            twist_msg.linear.z = mv[1]

            # Check if Balloon is still facing the target correctly, otherwise add angular
            # velocity to the Twist msg
            target_angle = math_utils.angle_between_points(self.position, target)

            if not (target_angle - angle_eps < self.yaw < target_angle + angle_eps):
                angle_diff = (self.yaw - target_angle)
                twist_msg.angular = Vector3(x=0.0, y=0.0, z=math.sin(angle_diff))

            # Publish msg
            self.cmd_vel_publisher.publish(twist_msg)

            # Update position and distance after finishing
            distance = math_utils.point_distance(self.position, target)

        # After reaching the target, publish a stop msg
        self.cmd_vel_publisher.publish(self.stop_msg)


    # Initialize GUI windows for each balloon cache
    def init_gui(self):
        dpg.create_context()

        with dpg.window(label=f"Balloon {balloon_id} Sensor Data"):
            with dpg.table(header_row=True):
                dpg.add_table_column(label="Balloon")

                self.table_rows = []
                for _ in range(CACHE_SIZE):
                    with dpg.table_row():
                        cell = dpg.add_text("N/A")
                        self.table_rows.append(cell)

        dpg.create_viewport(title='Balloon Cache', width=100, height=280)
        dpg.setup_dearpygui()
        dpg.show_viewport()

        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

        dpg.destroy_context()

    # Update GUI windows with new data
    def update_gui(self):
        for i in range(len(self.balloon_data)):
            if i < len(self.table_rows):
                dpg.set_value(self.table_rows[i], self.balloon_data[i])


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()

    drone_controller = BalloonController()
    executor.add_node(drone_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    drone_controller.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()