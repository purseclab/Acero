import threading
import sys
sys.path.append('/home/acero/Downloads/carla10/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg')
import carla
import time
from util import *
from physicalconstraint import *

from multiprocessing import Process
from acero_main import *
from attack_logging import *
import random


weather = random_weather()

config = "/home/acero/autoslayer/mission_setup/C2.json"

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
# filter out bicycles and motorcycles
vehicles_bp = world.get_blueprint_library().filter('vehicle.*')
vehicles_bp = [x for x in vehicles_bp if int(x.get_attribute('number_of_wheels')) >= 4]

attack_vehicle_model = random.choice(vehicles_bp)

attack_pos = random_transform_vehicle(config, "Attack")
vic_pos = random_transform_vehicle(config, "Victim")

attack_speed = random_speed_vehicle(config, "Attack")
vic_speed = random.uniform(5, 7)

inittime = time.time()
# Initial the attack scene
attacker, victim, stopped_car = scene_init(config, weather, 
    attack_vehicle_model, attack_pos, attack_speed, vic_speed)

stop_pos = stopped_car[0].get_transform()
inittime = time.time() - inittime

focus(world.get_spectator(), victim)
# Spawn a stopped vehicle in front of the victim car


agent_list=stopped_car
agent_commands=[[(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)] ]
steptime=0.8

starttime = time.time()
attack_commands, vic_traj, att_traj, rewinding_details = trajectory_generation(attacker, victim, agent_list, agent_commands, steptime, 
        config, weather, attack_vehicle_model, attack_pos, attack_speed,vic_pos, vic_speed)

round = 0


print(attack_commands, traj)
endtime = time.time()
print("Time: ", endtime - starttime)

attack_car_detail = vehicle_details("tesla", attack_pos, attack_speed, traj)
victim_car_detail = vehicle_details("toyota", vic_pos, vic_speed, trajectory=None)
stopped_car_detail = vehicle_details('tesla', stop_pos, 0, trajectory=None)

agent_list = [stopped_car_detail]
setup = mission_setup(weather, agent_list, None)
duration = mission_duration(None, endtime-starttime)

from util import missionname
missionname = "C2"


attlogger("C2", ATTACK_SUCCESS, setup, duration, victim_car_detail, attack_car_detail, attack_commands, steptime, rewinding_details=rewinding_details)

os.system("pkill -f CarlaUE4")
shutdown_autoware()