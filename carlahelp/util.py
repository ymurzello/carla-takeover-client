'''
Copyright 2020, Rocky Liang, All rights reserved
'''

#utilities for interacting with carla
import carla
import math
import random
import numpy as np

EPSILON = 1e-6

def link_server():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    return client, world

def print_vehicles():
    client, world = link_server()
    blueprints = world.get_blueprint_library()
    vehicles = blueprints.filter('vehicle.*')
    for i in vehicles:
        print(i)
        print('')

def get_transform(vehicle_location, angle, d=6):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-10))

def rotz(x,y,theta):
    '''returns rotated x and y, angle input is in degrees'''
    therad = math.radians(theta)
    rot = np.array([[math.cos(therad),-math.sin(therad)],[math.sin(therad),math.cos(therad)]])
    p = np.array([[x],[y]])
    result = np.matmul(rot,p)

    return result[0,0], result[1,0]

def get_transform2(vehicle_transform, x=-2.0):
    '''
    used for jump2car, calcs transform for spectator cam
    '''
    location = vehicle_transform.location
    heading = vehicle_transform.rotation.yaw

	#xy in relative world coords where we want the spectator cam
    xr, yr = rotz(x,0,heading)
    #worlds coord location for the spectator
    location += carla.Location(xr, yr, 40)

    return carla.Transform(location, carla.Rotation(yaw=heading, pitch=-70))

def get_ego_transform():
    '''run when ego vehicle is the only vehicle in map'''
    client, world = link_server()

    vehicle_list = world.get_actors().filter('vehicle.*')
    while len(vehicle_list)==0:
        vehicle_list = world.get_actors().filter('vehicle.*')

    return vehicle_list[0].get_transform()

def jump2car(car=None):
    '''teleports server view to car'''
    client, world = link_server()
    if car==None:
        #world.tick()
        carlist = world.get_actors().filter('vehicle.*')

        if(len(carlist)!=0):
            car = carlist[0]
        else:
            print('No cars found on server')
            return 1

    cartrans = car.get_transform()
    #print(cartrans.rotation.yaw)

    spectator = world.get_spectator()
    spectator.set_transform(get_transform2(car.get_transform()))

    return 0

def top_down(trans, world):
    '''teleports server view to top down of given location'''
    spectator = world.get_spectator()
    trans_copy = carla.Transform()
    trans_copy = trans
    trans_copy.location.z += 100
    trans_copy.location.x -= 20
    trans_copy.rotation.pitch -= 85

    spectator.set_transform(trans_copy)

def angle_wrap(unwrapped):
    '''
    wraps angle between -180 to 180
    input & output in degrees
    '''
    wrapped = (unwrapped + 180) % 360
    if(wrapped<0):
        wrapped += 360
    return wrapped - 180

def bgra2np(img, height, width, output_rgb=True):
    '''
    takes in a carla image object (BGRA)
    converts to numpy array in RGB
    '''
    array = np.frombuffer(img.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (height,width,4))
    array = array[:,:,:3]
    if output_rgb:
        array = array[:,:,::-1]
    return array

def measure_forward_velocity(curr_vel, curr_rot, return_both=False):
    '''
    returns speed in x direction of chassis
    '''
    heading = curr_rot.yaw
    #print([heading, vel_global.x])
    vx, vy = rotz(curr_vel.x, curr_vel.y, -heading)
    if not return_both:
    	return vx
    else:
    	return vx, vy

def distance(loc1, loc2):
    '''euclidean distance, input are two locations'''
    x_sq = (loc1.x - loc2.x)**2
    y_sq = (loc1.y - loc2.y)**2
    return math.sqrt(x_sq + y_sq)

def random_point_circle(radius):
    '''sample random point in circle'''
    a = random.uniform(0, 2*math.pi)
    r = math.sqrt(random.uniform(0,1))*radius
    x = r*math.cos(a)
    y = r*math.sin(a)

    return x, y


def polar2cartesian(r,theta):
    '''translate polar to cartesian coordinates'''
    x = r*math.cos(math.radians(theta))
    y = r*math.sin(math.radians(theta))
    return x,y

def curvature(trans1, trans2):
    '''
    calculates curvature given two transforms
    flip signs at the end because unreal engine sign convention is left hand
    '''
    alpha = trans2.rotation.yaw - trans1.rotation.yaw
    dist = distance(trans1.location, trans2.location)
    return -2*(math.sin(math.radians(alpha/2))/(max(dist,EPSILON)))

def car_frame_deltas(car_trans,wp_loc):
    '''
    given car trans and waypoint location
    returns delta x and y in car frame
    '''
    #relative distance in world coordinates
    world_delta_x = wp_loc.x - car_trans.location.x
    world_delta_y = wp_loc.y - car_trans.location.y
    #rotate world delta to car coordinates
    world_car_yaw = car_trans.rotation.yaw
    therad = math.radians(world_car_yaw)
    #rel_delta_x, rel_delta_y = util.rotz(world_delta_x,world_delta_y,world_car_yaw)
    rot = np.array([[math.cos(therad),math.sin(therad)],[-math.sin(therad),math.cos(therad)]])
    result = np.matmul(rot, np.array([[world_delta_x], [world_delta_y]]))
    rel_delta_x = result[0,0]
    rel_delta_y = result[1,0]

    return rel_delta_x, rel_delta_y


def teleport_random(car, map, sp_id=None):
    '''
    teleport car to random spawnpoint
    can take spawnpoint index as well
    '''
    #get sp list and settle on an sp
    #sp list is a list of transforms
    sp_list = map.get_spawn_points()
    num_sp = len(sp_list)
    if sp_id==None:
        sp_id = np.random.choice(np.arange(num_sp))
    #limit sp_id to valid range
    sp_id = max(min(sp_id, num_sp-1),0)
    sp = sp_list[sp_id]

    #move car to that sp
    car.set_transform(sp)
    print('Teleported to spawn point {}'.format(sp_id), sp.location.x, sp.location.y)

    #return sp

if __name__=="__main__":
    jump2car()
