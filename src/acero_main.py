import json, os, time, threading, random, carla
from physicalconstraint import *
from util import *
import argparse # Add arguments options
from carla import Vector3D


def candidate_command_generation(counter, guide):

    # Random range sets for two cars
    directions = [(0.1, 0.4), (-0.1, 0.1), (-0.4, -0.1)]
    thros = [(-1/3, 1/3), (1/3, 1), (-1, -1/3)]

    # Candidates vehicle commands
    candidate_vehicle_commands = []

    # If this is the first command
    if counter == 0:
        # There should be randompoints ^^ 2 number of vehicle commands
        for dir in directions:
            steer = random.uniform(dir[0], dir[1])
            for thro in thros:
                throttle = random.uniform(thro[0], thro[1])
                candidate_vehicle_commands.append((throttle, steer))
    else:
        dir_indicator = guide[0]
        thro_indicator = guide[1]
        dir_range = range(0, 0)
        thro_range = range(0, 0)
        if dir_indicator > 0:
            dir_range = range(0, 2)
        else:
            dir_range = range(1, 3)

        if thro_indicator > 0:
            thro_range = range(1, 3)
        else:
            thro_range = range(0, 2)

        for i in dir_range:
            steer = random.uniform(directions[i][0], directions[i][1])
            for j in thro_range:
                throttle = random.uniform(thros[j][0], thros[j][1])
                candidate_vehicle_commands.append((throttle, steer))

    return candidate_vehicle_commands

def command_generation(counter, guide, history_commands, attacker, victim, agent_list, agent_commands, round, config, weather, attack_vehicle_model, attack_pos, attack_speed,vic_pos, vic_speed):
    # First generate a candidate command set
    candidate_commands = candidate_command_generation(counter, guide)
    #candidate_commands = candidate_commands[:1]
    #return candidate_commands[0], attacker, victim, agent_list, agent_commands
    # Run each of the candidate commands in the simulator and calculate the robustness
    robustness = []

    global REWIND
    rewinding_details = []
    # Extract command from agent commands for each agent based on the round number
    agent_commands_round = []
    for i in range(len(agent_list)):
        agent_commands_round.append(agent_commands[i][round])

    for command in candidate_commands:
        attacker, victim , agent_list, agent_commands = rewind_scene(attacker, victim, agent_list, agent_commands, history_commands, config, weather, attack_vehicle_model, attack_pos, attack_speed,vic_pos, vic_speed)
        rewind = exec_command(attacker, victim,  command, agent_list, agent_commands_round, 0.8)
        if not rewind:
            robustness.append(robustness_calculation(victim, agent_list[0], TTC = True, map = carla.Client('localhost', 2000).get_world().get_map()))
        if ATTACK_SUCCESS:
            return command, attacker, victim, agent_list, agent_commands, rewinding_details
        if REWIND:
            rewinding_details.append("round: " + str(len(history_commands)) + "\n Reason: collision with attacker")
            REWIND = False
        else:
            rewinding_details.append("round: " + str(len(history_commands)) + "\n Reason: violate physical constraints")

    if len(robustness) == 0:
        return candidate_commands[0], attacker, victim, agent_list, agent_commands, rewinding_details
    
    # Return the command with the lowest robustness
    return candidate_commands[robustness.index(min(robustness))], attacker, victim, agent_list, agent_commands, rewinding_details

def trajectory_generation(attacker, victim, agent_list, agent_commands, exectime, config, weather, attack_vehicle_model, attack_pos, attack_speed,vic_pos, vic_speed):
    # Generate a trajectory for the attack car
    traj = [get_state(attacker)]
    vic_traj, att_traj = [get_state(victim)], [get_state(attacker)]
    attack_commands = []
    counter = 0

    global ATTACK_SUCCESS, COLLISION_OBJECT
    rewinding_details = []
    guide = (1, -1)

    while not ATTACK_SUCCESS and counter < 5:

        # reset the enviorment (Note: No need to rewind because it will rewind in the command generation)
        # attacker, victim, agent_list, agent_commands = rewind_scene(attacker, agent_list, agent_commands, attack_commands)

        # Generate a new command
        print(counter)
        command, attacker, victim, agent_list, agent_commands, rewinding = command_generation(counter, guide, attack_commands, attacker, victim, agent_list, agent_commands, counter, config, weather, attack_vehicle_model, attack_pos, attack_speed,vic_pos, vic_speed)
        rewinding_details.append(rewinding)

        # Execute the new command (lowest robustness)
        agent_commands_round = []
        for i in range(len(agent_list)):
            agent_commands_round.append(agent_commands[i][counter])

        exec_command(attacker, victim, command, agent_list, agent_commands_round, exectime)

        # Save the state
        traj.append(get_state(attacker))
        attack_commands.append(command)

        # Update the guide
        if counter > 0:
            guide = [traj[counter][0] - traj[counter-1][0], traj[counter][1] - traj[counter-1][1]]

        # Update the counter
        counter += 1

    if ATTACK_SUCCESS:
        print("Attack Success")
        ATTACK_SUCCESS = False
        COLLISION_OBJECT = None

    
    for i in agent_list:
        if i is not None:
            i.destroy()

    global sensorlist

    for i in sensorlist:
        if i.is_alive:
            i.destroy()
        else:
            sensorlist.remove(i)

    attacker.destroy()
    global DRIVER, WEATHER, REWIND
    # DRIVER.close()
    # Restart the bridge
    shutdown_openpilot()
    victim.destroy()
    time.sleep(4)


    attacker, victim, npc = scene_init(config, weather, 
                                                        attack_vehicle_model, attack_pos, attack_speed, 
                                                        vic_speed)
    agent_list = npc
    vt, at = exec_history_commands(attacker, victim, attack_commands, agent_list, agent_commands, exectime, capture=True, recordtraj=True)
    time.sleep(0.8)
    
    vic_traj += vt
    att_traj += at
    print(ATTACK_SUCCESS)
    if ATTACK_SUCCESS:
        for i in range(len(agent_list)):
            if agent_list[i].id == COLLISION_OBJECT:
                COLLISION_OBJECT = i


    return attack_commands, vic_traj, att_traj, rewinding_details, inittime, ATTACK_SUCCESS


def rewind_scene(attacker, victim, agent_list, agent_commands, history_commands, config, weather, attack_vehicle_model, attack_pos, attack_speed,vic_pos, vic_speed):
    # Restart carla server// Disable for speed concern


    for i in agent_list:
        if i is not None:
            i.destroy()

    global sensorlist

    for i in sensorlist:
        if i.is_alive:
            i.destroy()
        else:
            sensorlist.remove(i)

    attacker.destroy()
    global DRIVER, WEATHER, REWIND
    # DRIVER.close()
    # Restart the bridge
    shutdown_openpilot()
    victim.destroy()
    time.sleep(4)


    REWIND = False


    # Reinitialize the scene
    attacker, victim, stoppedcar = scene_init(config, weather, 
                                                            attack_vehicle_model, attack_pos, attack_speed, 
                                                            vic_speed)

    # Execute the history commands
    agent_list = stoppedcar
    exec_history_commands(attacker, victim, history_commands, agent_list, agent_commands, 0.8)

    return attacker, victim, agent_list, agent_commands

def scene_init(config, weather, 
    attack_vehicle_model, attack_pos, attack_speed, vic_speed):
    """
    Read attack scene configuration from json file and initial the scene
    """
    # Read attack scene configuration from json file

    global inittime, WORLD
    inittimecurr = time.time()
    with open(config, 'r') as f:
        scene_config = json.load(f)


    world = None
    while world is None:
        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(8.0)
            world = client.load_world('Town03')
        except:
            print("Error: Cannot connect to the carla server")
            shutdown_carla()
            start_carla()
            
    

    start_openpilot()
    
    #world = client.get_world()
    
    global missionname
    if missionname is None:
        missionname = scene_config['MissionID']

    
    try:
        victim = world.get_actors().filter('vehicle.tesla.model3')[0]
    except:
        print("Error: Cannot find the victim car")
        shutdown_openpilot()
        start_openpilot()
        victim = world.get_actors().filter('vehicle.tesla.model3')[0]


    focus(world.get_spectator(), victim)

    npc_list = []


    
    # we reopen autoware if it does not run for 5 seconds
    start_time = time.time()
    while math.sqrt(victim.get_velocity().x**2 + victim.get_velocity().y**2 + victim.get_velocity().z**2) <= 0 or victim.get_transform().location.x <= -35:
        time.sleep(0.01)
        if time.time() - start_time > 15:
            logging.error("Autoware does not run for 15 seconds, restart it")
            shutdown_openpilot()
            world = client.load_world('Town03')
            time.sleep(5)
            start_openpilot()
            start_time = time.time()
            victim = world.get_actors().filter('vehicle.tesla.model3')[0]


    world.set_weather(weather) 


    for i in scene_config['NPC']:
        npc = world.try_spawn_actor(world.get_blueprint_library().filter(scene_config['NPC'][i][0])[0], eval(scene_config['NPC'][i][1]))
        npc.set_target_velocity(eval(scene_config['NPC'][i][2]))
        npc_list.append(npc)

    attacker = None
    while attacker is None:
        try:
            attacker = world.spawn_actor(attack_vehicle_model, attack_pos)
        except:
            print("Error: Cannot spawn the attacker car")
            attack_pos = carla.Transform(carla.Location(x=130.550003, y=56.840000, z=0.300000), carla.Rotation(pitch=0.000000, yaw=179.999756, roll=0.000000))
    
    print(vic_speed)
    victim.set_target_velocity(carla.Vector3D(vic_speed,0,0)) 
    attacker.set_target_velocity(carla.Vector3D(attack_speed,0,0))

    attack_collision_sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=attacker)
    victim_collision_sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), carla.Transform(), attach_to=victim)

    lane_sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.lane_invasion'),
                                        carla.Transform(), attach_to=victim)


    attack_collision_sensor.listen(lambda event: attacker_handler())
    victim_collision_sensor.listen(lambda event: victim_handler(event, attacker))

    from util import sensorlist
    

    sensorlist.append(attack_collision_sensor)
    sensorlist.append(victim_collision_sensor)
    sensorlist.append(lane_sensor)    
    

    inittimecurr = time.time() - inittimecurr
    inittime += inittimecurr

    return attacker, victim, npc_list

def victim_handler(event, attacker):
    '''
    Victim collision handler
    Register in main file
    '''
    if event.other_actor.id == attacker.id:
        print("-------Collision with attacker-------")
    else:
        global ATTACK_SUCCESS
        ATTACK_SUCCESS = True
        print("-------Victim collision--------")
        global COLLISION_OBJECT
        COLLISION_OBJECT = event.other_actor.id

def attacker_handler():
    '''
    Attacker collision handler
    Register in main file
    '''
    global REWIND 
    REWIND = True
    print("-------Attacker collision--------")

def lane_function_handler(event, client):
    global LANE_INVASION, ATTACK_SUCCESS
    ATTACK_SUCCESS = True
    LANE_INVASION = True
    print("lane invasion")
