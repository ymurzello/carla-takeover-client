'''
scenario helpers for spawning npcs deterministically

Rocky Liang, 2020
'''
import random
import json
import argparse
import carla

from carlahelp import util
from carlahelp.filehelp import save_as_json

def generate_front_back_wp(ego_transform, map, n_front, n_back, dist, spawn_points=None):
    '''
    generate spawn waypoints for spawning npcs in front and back of the ego vehicle

    ego_transform: Carla.Transform object of the ego vehicle
    map: Carla.Map object
    n_front & n_back: number of cars in the front and back of ego to spawn
    dist: how far apart they are
    spawn_points: list of waypoints to spawn
    '''
    ego_wp = map.get_waypoint(ego_transform.location)

    if spawn_points==None:
        spawn_points = []

    #back loop
    dist_accumulate = 0
    for bi in range(n_back):
        dist_accumulate -= dist
        #new_wp_list = ego_wp.next(dist_accumulate)
        dx_rotated, dy_rotated = util.rotz(dist_accumulate, 0, ego_transform.rotation.yaw)
        back_location = ego_wp.transform.location + carla.Location(x=dx_rotated, y=dy_rotated)
        back_wp = map.get_waypoint(back_location)
        spawn_points.append(back_wp)

    #front loop
    curr_wp = ego_wp
    for fi in range(n_front):
        curr_wp = curr_wp.next(dist)[0]
        spawn_points.append(curr_wp)

    return spawn_points

def sparse_locations(r, n, separation, locations=None):
    '''helper function for location proposals'''
    if locations == None:
        locations = []
    while len(locations) < n:
        x_new, y_new = util.random_point_circle(r)
        loc_new = carla.Location(x=x_new, y=y_new)

        valid_location_flag = False
        while True:
            for idx, loc in enumerate(locations):
                if util.distance(loc, loc_new) < separation:
                    #too close, resample and restart for loop
                    x_new, y_new = util.random_point_circle(r)
                    loc_new = carla.Location(x=x_new, y=y_new)
                    break
                if idx == len(locations)-1:
                    #far enough away from all others, good to go
                    valid_location_flag = True

            if valid_location_flag == True or len(locations)==0:
                locations.append(loc_new)
                break
    return locations

def generate_npc_car_wp(ego_transform, map, r, n_cars, spawn_points=None):
    '''
    generate scattered spawnpoints for npc vehicles
    returns a list of carla.Waypoints objects

    ego_transform: spawn point of ego vehicle
    map: carla.Map object
    r: radius from ego vehicle within which to spawn npcs
    n_cars: amount of npc car spawn points to generate
    spawn_points: list of waypoints to spawn

    TODO: print amount asked vs amount generated
    '''

    if spawn_points==None:
        spawn_points = []

    n_cars_preexisting = len(spawn_points)
    #populate locations list for distance checking
    locations = [sp.transform.location for sp in spawn_points]
    #randomly sample locations within given radius
    locations = sparse_locations(r, n_cars, 8, locations)


    #shift each location to ego transform and determine if good
    for loc in locations:
        loc = loc + ego_transform.location
        wp_new = map.get_waypoint(loc)

        for idx, sp in enumerate(spawn_points):
            if util.distance(sp.transform.location, wp_new.transform.location) < 6:
                break
            if idx==len(spawn_points)-1:
                spawn_points.append(wp_new)

        if len(spawn_points)==0:
            spawn_points.append(wp_new)

    print('{} cars requested, {} cars generated'.format(n_cars, len(spawn_points)-n_cars_preexisting))

    return spawn_points

def generate_npc_ped_wp(ego_transform, map, r, n_peds, spawn_points=None):
    '''
    generate scattered spawnpoints for npc pedestrians
    returns a list of carla.Waypoint objects

    ego_transform: spawn point of ego vehicle
    map: carla.Map object
    n_peds: amount of npc pedestrian spawn points to generate
    spawn_points: list of waypoints to spawn
    '''
    if spawn_points==None:
        spawn_points = []

    n_peds_preexisting = len(spawn_points)
    #populate locations list for distance checking
    locations = [sp.transform.location for sp in spawn_points]
    #randomly sample locations within given radius
    locations = sparse_locations(r, n_peds, 8, locations)


    for loc in locations:
        loc = loc + ego_transform.location
        wp_new = map.get_waypoint(loc, lane_type=carla.LaneType.Sidewalk)
        for idx, sp in enumerate(spawn_points):
            if util.distance(sp.transform.location, wp_new.transform.location) < 8:
                break
            if idx==len(spawn_points)-1:
                spawn_points.append(wp_new)
        if len(spawn_points)==0:
            spawn_points.append(wp_new)

    print('{} peds requested, {} peds generated'.format(n_peds, len(spawn_points)-n_peds_preexisting))
    return spawn_points

def pair_sp_with_bp(spawn_points, actor_pattern, blueprint_library, world):
    '''given list of spawn points, return list of (spawn point, actor)
    or if pedestrian, (spawn point, actor, target, speed)
    basically gives you all the info that's needed to spawn actors

    spawn_points: list of spawn points
    actor_pattern: wildcard pattern for type of actor you want
    blueprint_library: carla.BlueprintLibrary object, holds all actor blueprints
    world: carla.World object
    '''
    spawn_info = []
    for sp in spawn_points:
        #randomly choose actor bp
        bp = random.choice(blueprint_library.filter(actor_pattern))

        if actor_pattern == "walker.pedestrian.*":
            #if pedestrians, randomly choose target
            target = world.get_random_location_from_navigation()
            tup = (sp, bp, target, 1+random.random())
        else:
            tup = (sp, bp)

        spawn_info.append(tup)

    return spawn_info

def compile_spawn_info(spawn_info):
    '''returns dict of spawn info for saving

    spawn_info: list of tuples generated by pair_sp_with_bp function
    '''
    data = {}
    #data[spawn_type] = {}
    for idx, tup in enumerate(spawn_info):
        for item in tup:
            if str(type(item))=="<class 'carla.libcarla.Waypoint'>":
                #process waypoint
                data[str(idx)] = {}
                data[str(idx)]['id'] = str(item.id)
                data[str(idx)]['road_id'] = str(item.road_id)
                data[str(idx)]['section_id'] = str(item.section_id)
                data[str(idx)]['lane_id'] = str(item.lane_id)
                data[str(idx)]['s'] = str(item.s)
                data[str(idx)]['x'] = str(item.transform.location.x)
                data[str(idx)]['y'] = str(item.transform.location.y)
                data[str(idx)]['z'] = str(item.transform.location.z)
                data[str(idx)]['pitch'] = str(item.transform.rotation.pitch)
                data[str(idx)]['yaw'] = str(item.transform.rotation.yaw)
                data[str(idx)]['roll'] = str(item.transform.rotation.roll)

            elif str(type(item))=="<class 'carla.libcarla.ActorBlueprint'>":
                #process blueprint
                data[str(idx)]['actor_type'] = item.id
            elif str(type(item))=="<class 'carla.libcarla.Location'>":
                #process target location
                data[str(idx)]['t_x'] = str(item.x)
                data[str(idx)]['t_y'] = str(item.y)
                data[str(idx)]['t_z'] = str(item.z)

            elif str(type(item))=="<class 'float'>":
                #process speed
                data[str(idx)]['walker_speed'] = str(item)

    return data

def finalize_spawn_info(car_spi, ped_spi, ego_wp, map):
    '''combine car and pedestrian dicts, add map/world info
    returns final dict to be serialized and saved as json file

    car_spi: output of compile_spawn_info function for cars
    ped_spi: output of compile_spawn_info function for pedestrians
    ego_wp: waypoint of ego spawn
    map: carla.Map object
    '''

    data = {}
    data['cars'] = car_spi
    data['walkers'] = ped_spi
    data['map'] = map.name

    data['ego_actor'] = {}

    #ego blueprint id
    data['ego_actor']['actor_type'] = 'vehicle.tesla.model3'

    #ego spawn location info, same as big list of shit above
    data['ego_actor']['id'] = str(ego_wp.id)
    data['ego_actor']['road_id'] = str(ego_wp.road_id)
    data['ego_actor']['section_id'] = str(ego_wp.section_id)
    data['ego_actor']['lane_id'] = str(ego_wp.lane_id)
    data['ego_actor']['s'] = str(ego_wp.s)
    data['ego_actor']['x'] = str(ego_wp.transform.location.x)
    data['ego_actor']['y'] = str(ego_wp.transform.location.y)
    data['ego_actor']['z'] = str(ego_wp.transform.location.z)
    data['ego_actor']['pitch'] = str(ego_wp.transform.rotation.pitch)
    data['ego_actor']['yaw'] = str(ego_wp.transform.rotation.yaw)
    data['ego_actor']['roll'] = str(ego_wp.transform.rotation.roll)

    return data


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-cr', '--car-radius', type=int, default=70)
    parser.add_argument('-cn', '--car-num', type=int, default=25)
    parser.add_argument('-es', '--ego-spawn', type=int, default=247)
    parser.add_argument('-fn', '--front-num', type=int, default=0)
    parser.add_argument('-rn', '--rear-num', type=int, default=0)
    parser.add_argument('-pn', '--ped-num', type=int, default=4)
    parser.add_argument('-pr', '--ped-radius', type=int, default=30)
    args = parser.parse_args()

    client, world = util.link_server()
    map = world.get_map()
    bpl = world.get_blueprint_library()
    sp = map.get_spawn_points()[args.ego_spawn]

    util.top_down(sp, world)

    r = args.car_radius
    num_cars = args.car_num
    num_peds = args.ped_num
    #put ego spawn waypoint in wps list so no npc spawn there
    car_wps = [map.get_waypoint(sp.location)]
    car_wps = generate_front_back_wp(sp, map, args.front_num,args.rear_num,15, car_wps)
    car_wps = generate_npc_car_wp(sp, map, r, num_cars, car_wps)

    ped_wps = generate_npc_ped_wp(sp, map, args.ped_radius, num_peds)
    for wp in car_wps:
        world.debug.draw_point(wp.transform.location, life_time=5)
    for wp in ped_wps:
        world.debug.draw_point(wp.transform.location, color=carla.Color(b=255,g=80), life_time=5)
    world.debug.draw_point(car_wps[0].transform.location, color=carla.Color(g=255), life_time=5)

    #pop first from from car wps becaue that's the ego spawn
    ego_wp = car_wps.pop(0)

    #generate tuples with spawn info
    car_pairs = pair_sp_with_bp(car_wps, 'vehicle.*', bpl, world)
    ped_pairs = pair_sp_with_bp(ped_wps, 'walker.pedestrian.*', bpl, world)

    #convert tuples to dict
    cd = compile_spawn_info(car_pairs)
    pd = compile_spawn_info(ped_pairs)

    final_dict = finalize_spawn_info(cd, pd, ego_wp, map)
    save_as_json('test3.json',final_dict)
