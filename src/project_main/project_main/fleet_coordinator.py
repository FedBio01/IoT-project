import sys
from time import sleep
from threading import Thread
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from project_interfaces.action import Patrol

NUMBER_OF_BALLOONS = int(sys.argv[1])
SIMULATION_SIDE = int(sys.argv[2])
HOVERING_HEIGHT = 15.0

class FleetCoordinator(Node):

    """
    Fleet Coordinator class, used the manage the whole fleet of Balloons and Drones.
    This is where most of the task should be submitted by default.
    """

    def __init__(self):

        super().__init__('fleet_coordinator')

        self.balloon_action_clients = {}
        self.balloon_positions = {}
        self.balloon_states = {}
        self.distances = self.calculate_distances(NUMBER_OF_BALLOONS, SIMULATION_SIDE)

        for i in range(NUMBER_OF_BALLOONS):
            self.balloon_action_clients[i] = ActionClient(
                self,
                Patrol,
                f'/Balloon_{i}/patrol'
            )
            self.balloon_states[i] = BalloonState.LANDED
            
            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda msg, id=i: self.store_balloon_position(id, msg),
                10
            )

    # function used to calculate the targets (expressed in x values) of the balloons
    def calculate_distances(self, number_of_balloons, area_side):

        # initialize the array that will contain the targets, 
        # that are expressed in terms of distance from the center of the area
        dist = [0] * number_of_balloons 
        
        # divide balloons in two halves
        # since we want them distributed symmetrically along the X-axis,
        # they are divided in two groups: one for positive X values and one for negative ones
        half = number_of_balloons // 2

        # if number of balloons is even, there is no balloon with X=0 (for symmetry)
        if number_of_balloons % 2 == 0:
            j = 1  # initialize numerator that expresses the factor to be multiplied by the area_side
            k = -1 # same as j but for negative values
            for i in range(half):
                dist[i] = j / number_of_balloons * area_side # formula inferred by geometrical considerations
                j += 2 # increment numerator for next balloons
            for i in range(half, number_of_balloons):
                dist[i] = k / number_of_balloons * area_side
                k -= 2
        
        # if number of balloons is odd, there is one with X=0
        else:
            j = 2  # initialization of numerators "= +/- 2" because of geometrical considerations
            k = -2
            for i in range(half):
                dist[i] = j / number_of_balloons * area_side
                j += 2
            for i in range(half + 1, number_of_balloons): # "half+1" so that we leave one balloon with "default" x=0
                dist[i] = k / number_of_balloons * area_side
                k -= 2

        return dist

    def patrol_targets(self):
        def patrol_targets_inner():
            while True:
                for i in range(NUMBER_OF_BALLOONS):
                    if self.balloon_states[i] != BalloonState.MOVING:
                        self.get_logger().info(f"Trying to submit task for Balloon {i}")
                        self.submit_task(i)
                sleep(10)  # avoid to overload the loop

        Thread(target=patrol_targets_inner).start()

    def submit_task(self, uav_id: int):
        while not self.balloon_action_clients[uav_id].wait_for_server(1):
            self.get_logger().info(f"Waiting for action server to come online for Balloon {uav_id}")
            sleep(3)

        if uav_id not in self.balloon_positions:
            self.get_logger().info(f"Waiting for Balloon {uav_id} position to be available")
            return

        self.balloon_states[uav_id] = BalloonState.MOVING

        goal = Patrol.Goal()
        goal.targets = []

        target_position = Point()
        target_position.x = float(self.distances[uav_id]) # calculate X target position for the balloon
        target_position.y = 0.0                           # keep balloons on X axis
        target_position.z = HOVERING_HEIGHT + (uav_id * 8.0)

        goal.targets.append(target_position)

        self.get_logger().info(f"Submitting task for Balloon {uav_id} to target x: {target_position.x}, y: {target_position.y}, z: {target_position.z}")

        patrol_future = self.balloon_action_clients[uav_id].send_goal_async(goal)
        patrol_future.add_done_callback(lambda future, uav_id=uav_id: self.patrol_submitted_callback(uav_id, future))

    def patrol_submitted_callback(self, uav_id, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info(f"Task has been refused by the action server for Balloon {uav_id}")
            self.balloon_states[uav_id] = BalloonState.HOVERING
            return

        self.get_logger().info(f"Task accepted for Balloon {uav_id}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future, uav_id=uav_id: self.patrol_completed_callback(uav_id, future))

    def patrol_completed_callback(self, uav_id, future):
        self.get_logger().info(f"Patrolling action for Balloon {uav_id} has been completed. Balloon is going idle")
        self.balloon_states[uav_id] = BalloonState.HOVERING

    def store_balloon_position(self, id, msg: Odometry):
        self.balloon_positions[id] = msg.pose.pose.position

class BalloonState(Enum):
    LANDED = 1
    HOVERING = 2
    MOVING = 3

def main():
    rclpy.init()

    executor = MultiThreadedExecutor()
    fleet_coordinator = FleetCoordinator()

    executor.add_node(fleet_coordinator)
    fleet_coordinator.patrol_targets()

    executor.spin()

    executor.shutdown()
    fleet_coordinator.destroy_node()
    rclpy.shutdown()
