import subprocess
import sys
import math

from random import randint

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction

from project_main.sim_utils import spawn_sdf




WORLD_NAME = "iot_project_world"

NUMBER_OF_BALLOONS = 3
NUMBER_OF_SENSORS = 6
SIMULATION_SIDE = 30
BASE_STATION_CENTER = -5

#-----------------------------------------------------------------------------------------------
# Launch file for the IoT Project. Launches all the nodes required to start the final solution
#-----------------------------------------------------------------------------------------------

def generate_launch_description():

    targets_to_spawn = []
    positions = [BASE_STATION_CENTER-1,BASE_STATION_CENTER,BASE_STATION_CENTER+1] #to avoid base station collisions
  


    for i in range(NUMBER_OF_BALLOONS):
        targets_to_spawn.append(spawn_sdf("resources/balloon/balloon.sdf", id = i, pos = (0, i*5, 0)))
    
    for i in range(NUMBER_OF_SENSORS):
        new_pos_x = randint(-SIMULATION_SIDE, SIMULATION_SIDE)
        new_pos_y = randint(-SIMULATION_SIDE, SIMULATION_SIDE)
        new_pos_z = 0
        while(new_pos_y in positions): new_pos_y= randint(-SIMULATION_SIDE, SIMULATION_SIDE)
        positions.append(new_pos_y)
        new_pos = new_pos_x, new_pos_y, new_pos_z
        targets_to_spawn.append(spawn_sdf("resources/active_sensor/active_sensor.sdf", id=i, pos=new_pos, rot=(0, 0, 0)))


    targets_to_spawn.append(spawn_sdf("resources/base_station/base_station.sdf", pos = (BASE_STATION_CENTER, BASE_STATION_CENTER, 0)))



    return LaunchDescription(targets_to_spawn)
