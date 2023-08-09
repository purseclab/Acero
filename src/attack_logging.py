import json
import numpy as np
import os
import sys
import time
from carla import Transform, Location, Rotation, WeatherParameters
from util import file_name

class mission_setup:
    '''
    1. Weather (cloudiness, precipitation, precipitation_deposits, sun_altitude_angle)
    2. Traffic (traffic vehicle starting positions, pedestrian starting positions)
    '''
    def __init__(self, weather, trafficlist, pedslist):
        self.weather = weather
        self.trafficlist = trafficlist
        self.pedslist = pedslist

class mission_duration:
    '''
    1. Scene initial time
    2. Path generation time
    '''
    def __init__(self, scene_initial_time, path_generation_time):
        self.scene_initial_time = scene_initial_time
        self.path_generation_time = path_generation_time

class vehicle_details:
    '''
    1. Vehicle model
    2. Starting position
    3. Initial speed
    4. Trajectory
    '''
    def __init__(self, model, starting_position, initial_speed, trajectory):
        self.model = model
        self.starting_position = starting_position
        self.initial_speed = initial_speed
        self.trajectory = trajectory
    


def attlogger(mission_id, mission_result, mission_setup, mission_duration, victim_car_details, attacker_car_details, attack_commands, attack_step_time, rewinding_details=None, collision_obj=None ):
    '''
    1. File name: time stamp
    2. Mission ID
    3. Mission result
    4. Mission Setup
    5. Mission duration (Scene initial time and path generation time)
    6. Rewinding details
    7. Victim car details
    8. Attacker car details
    9. Write to the folder based on the mission result
    '''

    log = {
        "mission_id": mission_id,
        "mission_result": mission_result,
        
        "mission_setup": {
            "weather": {
                "cloudiness": mission_setup.weather.cloudiness,
                "precipitation": mission_setup.weather.precipitation,
                "precipitation_deposits": mission_setup.weather.precipitation_deposits,
                "sun_altitude_angle": mission_setup.weather.sun_altitude_angle
            },
            "traffic": {
                "pedestrian_starting_positions": str(mission_setup.pedslist)
            }
        },

        "mission_duration": {
            "scene_initial_time": mission_duration.scene_initial_time,
            "path_generation_time": mission_duration.path_generation_time
        },
        
        "rewinding_details": rewinding_details,

        "victim_car_details": {
            "model": str(victim_car_details.model),
            "starting_position": str(victim_car_details.starting_position),
            "initial_speed": str(victim_car_details.initial_speed),
            "trajectory": victim_car_details.trajectory
        },

        "attacker_car_details": {
            "model": attacker_car_details.model,
            "starting_position": str(attacker_car_details.starting_position),
            "initial_speed": attacker_car_details.initial_speed,
            "trajectory": attacker_car_details.trajectory
        },

        "attack_commands": attack_commands,
        "attack_step_time": attack_step_time,
        "collision_object": collision_obj
    }

    log["mission_setup"]["traffic"]["traffic_vehicle_starting_positions"] = []
    log["mission_setup"]["traffic"]["traffic_vehicle_speed"] = []
    log["mission_setup"]["traffic"]["model"] = []


    for agent in mission_setup.trafficlist:
        log["mission_setup"]["traffic"]["traffic_vehicle_starting_positions"].append(str(agent.starting_position))
        log["mission_setup"]["traffic"]["traffic_vehicle_speed"].append(str(agent.initial_speed))
        log["mission_setup"]["traffic"]["model"].append(str(agent.model))


    file_name = time.strftime("%d-%H-%M", time.localtime())


    if mission_result:
        file_name =  "/home/cps/Documents/mrdata/" + mission_id + "/log/success/" + file_name + ".json"
    else:
        file_name =  "/home/cps/Documents/mrdata/" + mission_id + "/log/failure/" + file_name + ".json"


    with open(file_name, 'w') as f:
        json.dump(log, f, indent=4)