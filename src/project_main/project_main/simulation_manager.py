import sys
import math
import threading
import queue

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math_utils
import dearpygui.dearpygui as dpg

NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])
SIMULATION_SIDE = int(sys.argv[3])

# higher balloon position
p0 = Point()
p0.x = 0.0
p0.y = float(SIMULATION_SIDE*(NUMBER_OF_BALLOONS-1)/NUMBER_OF_BALLOONS) # inferred by geometrical considerations
p0.z = 15.0 + (NUMBER_OF_BALLOONS-1)*8.0

# worst position for a sensor (angle of the area)
p1 = Point()
p1.x = float(SIMULATION_SIDE)
p1.y = float(SIMULATION_SIDE)
p1.z = 0.0

# calculate the sensor range based on the distance between the higher balloon 
# and the worst position in the are for a sensor
SENSORS_RANGE = math_utils.point_distance(p0, p1)

class SimulationManager(Node):

    def __init__(self):
        super().__init__('simulation_manager')

        self.sensor_positions = {}
        self.balloon_positions = {}

        for i in range(NUMBER_OF_SENSORS):
            self.create_subscription(
                Odometry,
                f'ActiveSensor_{i}/odometry',
                lambda odometry_msg, sensor_id=i: self.store_sensor_position(sensor_id, odometry_msg),
                10
            )

            self.create_subscription(
                String,
                f'ActiveSensor_{i}/tx_data',
                lambda string_msg, sensor_id=i: self.forward_data(sensor_id, string_msg),
                10
            )

        self.balloons_rx = {}
        self.balloon_caches = {} # dictionary fir balloon caches
        self.found_balloons = []
        self.missed_balloons = []

        for i in range(NUMBER_OF_BALLOONS):
            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda odometry_msg, balloon_id=i: self.store_balloon_position(balloon_id, odometry_msg),
                10
            )

            self.balloons_rx[i] = self.create_publisher(
                String,
                f'Balloon_{i}/rx_data',
                10
            )

            # topic to receive caches from balloons
            self.create_subscription(
                String,
                f'Balloon_{i}/tx_data',
                lambda string_msg, balloon_id=i: self.tx_callback(balloon_id, string_msg),
                10
            )

        # topic to send responses to the base station
        self.base_station_rx = self.create_publisher(
            String,
            'base_station/rx_data',
            10
        )

        # topic to receive requests from the base station
        self.create_subscription(
            String,
            'base_station/requests',
            lambda string_msg: self.requests(string_msg),
            NUMBER_OF_SENSORS*3 # base station sends 3 requests per sensor
        )

        # set of variables for GUI statistics
        self.total_requests = 0 #total number of requests
        self.total_cache_misses = 0 #cache misses by all balloons
        self.total_found_values = 0 #values found by all balloons
        self.satisfied_requests = 0 #requests with at least 1 found value
        self.total_poisson = 0 #total number of poisson requests
        self.total_gaussian = 0 #total number of gaussian requests
        self.total_random = 0 #total number of random requests
        self.satisfied_poisson = 0 #satisfied poisson requests
        self.satisfied_gaussian = 0 #satisfied gaussian requests
        self.satisfied_random = 0 #satisfied random requests

        self.gui_queue = queue.Queue()

        self.gui_thread = threading.Thread(target=self.init_gui)
        self.gui_thread.start()

    def store_sensor_position(self, sensor_id, position: Odometry):
        self.sensor_positions[sensor_id] = position.pose.pose.position

    def store_balloon_position(self, balloon_id, position: Odometry):
        self.balloon_positions[balloon_id] = position.pose.pose.position

    def forward_data(self, sensor_id, msg: String):
        for i in range(NUMBER_OF_BALLOONS):
            if sensor_id in self.sensor_positions and i in self.balloon_positions:
                if math_utils.point_distance(self.sensor_positions[sensor_id], self.balloon_positions[i]) < SENSORS_RANGE:
                    self.balloons_rx[i].publish(msg)

    # receive cache from balloons
    def tx_callback(self, balloon_id, msg: String):
        new_cache = msg.data.split(" ")
        self.balloon_caches[balloon_id] = new_cache
        self.get_logger().info(f"Cache attuali: {self.balloon_caches}")

    # manage base station requests
    def requests(self, msg: String):
        self.get_logger().info(f"requests: {msg.data}")

        # 1. check which balloons have the requested data
        self.check_caches_requests(msg)

        # 2. update GUI statistics 
        self.update_statistics(msg)
        self.gui_queue.put((self.total_requests, self.total_cache_misses, self.total_found_values, self.satisfied_requests, self.total_poisson, self.total_gaussian, self.total_random, self.satisfied_poisson, self.satisfied_gaussian, self.satisfied_random))

        # 3. publish results to the base station
        msg = String()
        msg.data = f"found_values: {self.found_balloons} + missed_values: {self.missed_balloons}"
        self.base_station_rx.publish(msg)



    def check_caches_requests(self, msg: String):
        request = msg.data
        self.found_balloons.clear()
        for balloon_id, caches in self.balloon_caches.items():
            if request in caches:
                self.found_balloons.append(balloon_id)

    def update_statistics(self, msg: String):
        self.total_requests += 1
        request = msg.data
        sensor_id = int(request.split("_")[0])
        if(sensor_id < NUMBER_OF_SENSORS//3):
            self.total_poisson += 1
        elif(NUMBER_OF_SENSORS//3 <= sensor_id < 2*NUMBER_OF_SENSORS//3 or NUMBER_OF_SENSORS == 1): 
            self.total_gaussian += 1
        elif(sensor_id >= 2*NUMBER_OF_SENSORS//3):
            self.total_random += 1

        self.missed_balloons = list(set(self.balloon_caches.keys()) - set(self.found_balloons))
        self.total_cache_misses += len(self.missed_balloons)
        self.total_found_values += len(self.found_balloons)
        if(len(self.missed_balloons) != NUMBER_OF_BALLOONS):
            self.satisfied_requests += 1
            if(sensor_id < NUMBER_OF_SENSORS//3):
                self.satisfied_poisson += 1
            elif(NUMBER_OF_SENSORS//3 <= sensor_id < 2*NUMBER_OF_SENSORS//3 or NUMBER_OF_SENSORS == 1): 
                self.satisfied_gaussian += 1
            elif(sensor_id >= 2*NUMBER_OF_SENSORS//3):
                self.satisfied_random += 1

    def init_gui(self):
        dpg.create_context()

        with dpg.window(label="Sensor Data Window"):
            with dpg.plot(label="Sensor Data Plot", height=400, width=600):
                dpg.add_plot_legend()

                x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Requests")
                y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="Values")

                self.cache_misses_series = dpg.add_line_series([], [], label="Cache Misses", parent=y_axis) 
                self.found_values_series = dpg.add_line_series([], [], label="Values Found", parent=y_axis) 

            self.total_requests_text = dpg.add_text(f"Total Requests: {self.total_requests}")
            self.total_misses_text = dpg.add_text(f"Total Cache Misses: {self.total_cache_misses}")
            self.total_found_text = dpg.add_text(f"Total Values Found: {self.total_found_values}")
            self.requests_ratio_text = dpg.add_text(f"Satisfied requests: N/A")
            self.satisfied_gaussian_text = dpg.add_text(f"Satisfied Gaussian Requests: N/A")
            self.satisfied_poisson_text = dpg.add_text(f"Satisfied Poisson Requests: N/A")
            self.satisfied_random_text = dpg.add_text(f"Satisfied Random Requests: N/A")

        dpg.create_viewport(title='Sensor Data Plot', width=800, height=600)
        dpg.setup_dearpygui()
        dpg.show_viewport()

        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

            while not self.gui_queue.empty():
                total_requests, total_cache_misses, total_found_values, satisfied_requests, total_poisson, total_gaussian, total_random, satisfied_poisson, satisfied_gaussian, satisfied_random = self.gui_queue.get()
                self.update_gui(total_requests, total_cache_misses, total_found_values, satisfied_requests, total_poisson, total_gaussian, total_random, satisfied_poisson, satisfied_gaussian, satisfied_random)

        dpg.destroy_context()

    def update_gui(self, total_requests, total_cache_misses, total_found_values, satisfied_requests, total_poisson , total_gaussian , total_random, satisfied_poisson, satisfied_gaussian, satisfied_random):
        # Get current series data
        current_cache_misses_data = dpg.get_value(self.cache_misses_series)
        current_found_values_data = dpg.get_value(self.found_values_series)

        # Generate new points
        cache_misses_x = [current_cache_misses_data[0][-1] if current_cache_misses_data[0] else 0, total_requests]
        cache_misses_y = [current_cache_misses_data[1][-1] if current_cache_misses_data[1] else 0, total_cache_misses]

        found_values_x = [current_found_values_data[0][-1] if current_found_values_data[0] else 0, total_requests]
        found_values_y = [current_found_values_data[1][-1] if current_found_values_data[1] else 0, total_found_values]

        # Update series
        dpg.set_value(self.cache_misses_series, [current_cache_misses_data[0] + cache_misses_x, current_cache_misses_data[1] + cache_misses_y])
        dpg.set_value(self.found_values_series, [current_found_values_data[0] + found_values_x, current_found_values_data[1] + found_values_y])

        # Update text
        dpg.set_value(self.total_requests_text, f"Total Requests: {total_requests}")
        dpg.set_value(self.total_misses_text, f"Total Cache Misses: {total_cache_misses}")
        dpg.set_value(self.total_found_text, f"Total Values Found: {total_found_values}")
        dpg.set_value(self.requests_ratio_text, f"Satisfied requests: {round(satisfied_requests / total_requests * 100, 2)}%")
        if(total_gaussian != 0):
            dpg.set_value(self.satisfied_gaussian_text, f"Satisfied Gaussian Requests: {round(satisfied_gaussian / total_gaussian * 100, 2)}%")
        else:
            dpg.set_value(self.satisfied_gaussian_text, f"Satisfied Gaussian Requests: N/A")
        if(total_poisson != 0):
            dpg.set_value(self.satisfied_poisson_text, f"Satisfied Poisson Requests: {round(satisfied_poisson / total_poisson * 100, 2)}%")
        else:
            dpg.set_value(self.satisfied_poisson_text, f"Satisfied Poisson Requests: N/A")
        if(total_random != 0):
            dpg.set_value(self.satisfied_random_text, f"Satisfied Random Requests: {round(satisfied_random / total_random * 100, 2)}%")
        else:
            dpg.set_value(self.satisfied_random_text, f"Satisfied Random Requests: N/A")

def main():
    rclpy.init()

    simulation_manager = SimulationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(simulation_manager)

    executor.spin()
    executor.shutdown()
    simulation_manager.destroy_node()
    rclpy.shutdown()

