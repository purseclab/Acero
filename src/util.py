import sys
sys.path.append('/home/cps/Downloads/carla11/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg')

import carla, time, os, json
from threading import Thread
from math import sqrt, inf
from carla import Transform, Location, Rotation, WeatherParameters
import random
import numpy as np
import time, math, logging
from physicalconstraint import *
import threading
import subprocess, signal

inittime = 0

ATTACK_SUCCESS = False
REWIND = False
COLLISION_OBJECT = None
WORLD = None
LANE_INVASION = False

START_CARLA = "/home/acero/Downloads/carla11/CarlaUE4.sh -quality-level=Low 2>/dev/null &"
START_AUTOWARE = "docker exec -u autoware -i autoware bash -c \"bash /home/autoware/runautoware.sh&>/dev/null\" &"
SHUTDOWN_AUTOWARE = "docker exec -i -u autoware autoware pkill -f \"/usr/bin/python /opt/ros/melodic/bin/roslaunch carla_autoware_agent carla_autoware_agent.launch\""
SHUTDOWN_CARLA = "pkill -f CarlaUE4"
SHUTDOWN_OPENPILOT = "pkill -f bridge.py"

START_OPENPILOT = "/home/acero/Downloads/openpilot/tools/sim/bridge.py --init True >/dev/null &"
OPPROCESSES = ['bridge.py']

file_name = None
missionname = None
sensorlist = []


def start_autoware():
    os.system(START_AUTOWARE)
    time.sleep(10)


def start_carla():
    os.system(START_CARLA)
    time.sleep(5)

def shutdown_carla():
    os.system(SHUTDOWN_CARLA)
    time.sleep(1)


def shutdown_autoware():
    os.system(SHUTDOWN_AUTOWARE)
    time.sleep(1)

def start_openpilot():
    os.system(START_OPENPILOT)
    time.sleep(5)

def shutdown_openpilot():
    os.system(SHUTDOWN_OPENPILOT)
    time.sleep(4)


def random_weather():
    """
    Randomly generate weather parameters based on a data-driven statistical model
    """

    sunA = 90 * random.uniform(0,1)
    pg = 50 * random.uniform(0,1)
    cb = np.random.beta(2, 2)
    cu = random.uniform(0,1)
    c = 0
    pa = 0

    if cu < 0.5:
        c = 30 * cb
    else:
        c = 40 * cb + 60

    if c >= 70:
        pa = c

    weather = carla.WeatherParameters(
        cloudiness=c,
        precipitation=pa,
        precipitation_deposits=pg,
        sun_altitude_angle=sunA,) 

    return weather


def random_transform_vehicle(config, vehicle):

    with open(config, 'r') as f:
        scene_config = json.load(f)

    offset_x = 0
    offset_y = 0

    if random.choice([True, False]):
        offset_x = random.uniform(-3,-5)
    else:
        offset_x = random.uniform(-3,-5)

    #if random.choice([True, False]):
    #    offset_y = random.uniform(1, 3)
    #else:
    #    offset_y = -random.uniform(1, 3)

    transform = eval(scene_config[vehicle+"Transform"])
    transform.location.x += offset_x
    transform.location.y += offset_y

    return transform

def random_speed_vehicle(config, vehicle):
    
    with open(config, 'r') as f:
        scene_config = json.load(f)

    speed = random.uniform(scene_config[vehicle+"Speed"]-2, scene_config[vehicle+"Speed"]+2)
    return speed




def exec_history_commands(attacker, victim, history_commands, agent_list, agent_commands_list, duration, capture=False, recordtraj=False):
    # Input: history_commands, actor_list, time
    # History_commands: a 2d list of history list of carla vehicle commands
    round = len(history_commands)
    print(history_commands)
    vic_traj = []
    att_traj = []
    for i in range(round):
        agent_commands = []
        for j in range(len(agent_list)):
            agent_commands.append(agent_commands_list[j][i])
        print("agent_commands", agent_commands)
        exec_command(attacker, victim, history_commands[i], agent_list, agent_commands, duration, capture=capture)
        if recordtraj:
            vic_traj.append(get_state(victim))
            att_traj.append(get_state(attacker))
        
    return vic_traj, att_traj

def exec_command(attacker, victim, history_command, agent_list, agent_commands, duration, capture=False):

    # if capture is enabled, the function will capture the front camera image of victim and save it to the folder
    # the image name is the timestamp of the image
    cam_bp = None
    global file_name
    if capture:
        # --------------
        # Spawn attached RGB camera
        # --------------
        world = carla.Client('localhost', 2000).get_world()
        ego_cam = world.get_actors().filter('sensor.camera.rgb')[0]
        if file_name is None:
            file_name = time.strftime("%d-%H-%M", time.localtime())
        
        print("file_name", file_name)

        from acero_main import missionname

        ego_cam.listen(lambda image: image.save_to_disk('/home/acero/Documents/mrdata/'+missionname+ '/frontcam/' +file_name + '/'+'%.5d.jpg' % image.frame))
        # --------------

    print("Before Exec", robustness_calculation(victim, agent_list[0], TTC=True, DIST=False))
    attacker.apply_control(carla_command(history_command))
    for (actor, command) in zip (agent_list, agent_commands):
        actor.apply_control(carla_command(command))
    time.sleep(duration)
    print("After Exec", robustness_calculation(victim, agent_list[0], TTC=True, DIST=False))

    map = carla.Client('localhost', 2000).get_world().get_map()
    overspeed = check_os(attacker)
    wrongdirection = check_wd(attacker, map)

    print("Overspeed: ", overspeed)
    print("Wrong Direction: ", wrongdirection)
    global ATTACK_SUCCESS, REWIND
    print("Attack Success: ", ATTACK_SUCCESS)



    return overspeed or wrongdirection or REWIND

def robustness_calculation(acar, vcar, TTC=False, DIST=False, map = None):
    if TTC:
        ttc = time_to_collision(acar, vcar)
        if ttc == -1:
            return inf
        return ttc
    if DIST:
        dist = Dist(vcar, map)
        if dist == -1:
            return inf
        return dist



def get_state(actor):
    # Get the x and y coordinates of the actor
    actor_location = actor.get_location()
    return [actor_location.x, actor_location.y]

def carla_command(command):
    # Convert the command to carla command
    if command[0] < 0:  # brake
        return carla.VehicleControl(throttle=0, steer= command[1], brake=abs(command[0]))
    else:   # Throttle
        return carla.VehicleControl(throttle=command[0], steer=command[1], brake=0)


def time_to_collision(object_a, object_b):
    """
    Calculate the TTC of between two objects
    (Need to consider the direction of the object)
    """

    # 1. Decide which one is the front object. Here we assume the front direction is to 'positive_x' direction for simplicity
    if object_a.get_transform().location.x > object_b.get_transform().location.x:
        front_object = object_a
        back_object = object_b
    else:
        front_object = object_b
        back_object = object_a
    
    # 2. Decide which one is the left object. Here we assume the right direction is to 'positive_y' direction for simplicity
    if object_a.get_transform().location.y < object_b.get_transform().location.y:
        left_object = object_a
        right_object = object_b
    else:
        left_object = object_b
        right_object = object_a
    
    # 3. Decide the potential collision part
    most_left_point = right_object.get_transform().location.y - right_object.bounding_box.extent.y
    most_right_point = left_object.get_transform().location.y + left_object.bounding_box.extent.y

    most_front_point = back_object.get_transform().location.x + back_object.bounding_box.extent.x
    most_back_point = front_object.get_transform().location.x - front_object.bounding_box.extent.x


    different_direction = False
    if abs(front_object.get_transform().rotation.yaw - back_object.get_transform().rotation.yaw) > 0.45:
        different_direction = True
        most_back_point = front_object.get_transform().location.x - front_object.bounding_box.extent.y
        most_front_point = back_object.get_transform().location.x + back_object.bounding_box.extent.y

    # If collision is not possible, return -1
    # 1. Check if the front object is in the potential collision area
    # 2. Check if the back object has a higher speed
    if different_direction and front_object.get_velocity().x < back_object.get_velocity().x:
        return (most_back_point-most_front_point)/(back_object.get_velocity().x - front_object.get_velocity().x)
    elif most_left_point < most_right_point or front_object.get_velocity().x >= back_object.get_velocity().x:
        return -1
    else:
        return (most_back_point-most_front_point)/(back_object.get_velocity().x - front_object.get_velocity().x)


def Dist(obj, map):
    """
    Calculate the distance between two objects
    """

    # 1. Get the nearst way point of vehicle
    obj_waypoint = map.get_waypoint(obj.get_location(), project_to_road=True)
    
    # 2. Calculate the distance between the vehicle and the cloest lane mark
    dist = obj_waypoint.lane_width/2 - abs(obj.get_location().y - obj_waypoint.transform.location.y)

    return dist
    
def distance(point_a, point_b):
    return sqrt((point_a[0]-point_b[0])**2 + (point_a[1]-point_b[1])**2)

def focus(camera, obj):
    """
    Focus spectator camera on the object
    """
    transform = obj.get_transform()
    camera.set_transform(carla.Transform(transform.location + carla.Location(x=-6, z=3), transform.rotation))

def draw_boundingbox(debug, vehicle):
    """
    Draw the bounding box for the input vehicle
    """
    boundingbox = vehicle.bounding_box
    boundingbox.location += vehicle.get_transform().location
    debug.draw_box(boundingbox, vehicle.get_transform().rotation,
             thickness=0.1, color=carla.Color(255, 0, 0), life_time = 0)

