import os
import threading
import time
import math
import logging
from util import *
#logging.getLogger().setLevel(logging.INFO)

ATTACK_SUCCESS = False

class PhysicalConstraint:
    """
    PhysicalConstraint class
    """
    def __init__(self):
        self.two_feet = False # PC1 Enforced at the input side of the attacker 
        self.victim_collision = False  #PC2 Collision sensor in main file
        self.attacker_collision = False  #PC3   Collision sensor in main file
        self.lanechangebaddistance = False  #PC4
        self.followbaddistance = False  #PC5
        self.overspeed = False #PC6
        self.wrong_direction = False  #PC7


def over_speed(vehicle, violation):
    """
    Check weather the vehicle is over speed
    """
    # calculate the magnitude of the speed
    while True:
        speed = vehicle.get_velocity()
        speed_magnitude = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
        if speed_magnitude > 35:
            violation.overspeed = True
            logging.info("Vehicle is over speed %.4s", speed_magnitude)

def check_os(vehicle):
    """
    Check weather the vehicle is over speed
    """
    # calculate the magnitude of the speed
    speed = vehicle.get_velocity()
    speed_magnitude = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
    if speed_magnitude > 35:
        logging.info("Vehicle is over speed %.4s", speed_magnitude)
        return True
    return False

def lane_change_bad_distance(attacker, npc, threshold, violation):
    """
    Check weather the vehicle is too close to the front vehicle
    """
    # calculate the ttc between the two vehicles
    while True:
        ttc = time_to_collision(attacker, npc)
        if ttc < threshold:
            violation.lanechangebaddistance = True
            logging.info("Vehicle is doing a bad lane change%s", ttc)

def check_lcbd(victim, npc, threshold):
    """
    Check weather the vehicle is too close to the front vehicle
    """
    # calculate the ttc between the two vehicles
    ttc = time_to_collision(victim, npc)
    if ttc < threshold:
        logging.info("Vehicle is doing a bad lane change%s", ttc)
        return True
    return False


def follow_bad_distance(victim, front, threshod, violation):
    """
    Check weather the vehicle is too close to the front vehicle
    """
    # calculate the ttc between the two vehicles
    while True:
        ttc = time_to_collision(victim, front)
        if ttc < threshod:
            violation.followbaddistance = True
            logging.info("Vehicle is too close to the front vehicle")

def check_fbd(victim, front, threshod):
    """
    Check weather the vehicle is too close to the front vehicle
    """
    # calculate the ttc between the two vehicles
    ttc = time_to_collision(victim, front)
    if ttc < threshod:
        logging.info("Vehicle is too close to the front vehicle")
        return True
    return False


def wrong_direction(attacker, violation):
    """
    Check weather the vehicle is going the wrong direction
    """
    waypoint = map.get_waypoint(attacker.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
    if waypoint.lane_id < 0:
        violation.wrong_direction = True
        logging.info("Vehicle is going the wrong direction")

def check_wd(attacker, map):
    """
    Check weather the vehicle is going the wrong direction
    """
    waypoint = map.get_waypoint(attacker.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
    if waypoint.lane_id > 0:
        logging.info("Vehicle is going the wrong direction")
        return True
    return False

    

def victim_collision_handler(event, attacker):
    '''
    Victim collision handler
    Register in main file
    '''
    if event.other_actor.id == attacker.id:
        print("Collision with attacker")
    global ATTACK_SUCCESS
    ATTACK_SUCCESS = True
    print("-------Victim collision--------")

def attacker_collision_handler():
    '''
    Attacker collision handler
    Register in main file
    '''
    print("-------Attacker collision--------")