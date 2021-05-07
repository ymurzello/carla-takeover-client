'''
Copyright 2019, Rocky Liang, All rights reserved
'''
import carla
try:
    from carlahelp import util
except ModuleNotFoundError:
    import util

def simple_spawn(search=None):
    '''simple spawn of a car and camera'''
    #connect to client and retrieve world
    client, world = util.link_server()

    #get blueprints
    blueprint_library = world.get_blueprint_library()

    #select vehicle blueprint
    if search==None:
        search = 'vehicle.lincoln.*'
    vehicles = blueprint_library.filter(search)
    car_bp = vehicles[0]
    #car_bp.set_attribute('color','255,20,147')
    # car_bp.set_attribute('color','158,255,0')

    #select camera blueprint, set location
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    #sensor update dt
    cam_bp.set_attribute('sensor_tick', '0.05')
    cam_bp.set_attribute('image_size_x', '1280')
    cam_bp.set_attribute('image_size_y', '720')
    cam_trans = carla.Transform(carla.Location(x=-5, z=2.5), carla.Rotation(pitch=-10))

    #spawn vehicle and camera
    spawn_points = world.get_map().get_spawn_points()
    sp = spawn_points[27]

    car = world.spawn_actor(car_bp, sp)
    camera = world.spawn_actor(cam_bp, cam_trans, attach_to=car)
    #camera = world.try_spawn_actor(cam_bp, cam_trans, attach_to=car, attachment_type=carla.AttachmentType.SpringArm)
    print('Spawned car and spectator camera')

    return car, camera

def spawn_rgb(car=None, world=None, img_x=200, img_y=88, hz=7.463, pitch=-8):
    '''Spawns sensor on a car. Input is a vehicle actor object'''
    if world==None:
    	client, world = util.link_server()

    #get camera blueprint
    blueprints = world.get_blueprint_library()
    cam_bp = blueprints.find('sensor.camera.rgb')

    #set refresh rate, image size, and location on car
    if hz == 0:
        cam_bp.set_attribute('sensor_tick', str(0))
    else:
        cam_bp.set_attribute('sensor_tick', str(1.0/hz))
    cam_bp.set_attribute('image_size_x', str(img_x))
    cam_bp.set_attribute('image_size_y', str(img_y))
    relative_loc = carla.Location(x=2, z=1.4)
    relative_rot = carla.Rotation(pitch=pitch)
    cam_trans = carla.Transform(relative_loc,relative_rot)

    #spawn camera
    camera = world.spawn_actor(cam_bp, cam_trans, attach_to=car)
    print("Spawned rgb camera sensor")
    return camera

def spawn_ss(car=None, world=None, img_x=200, img_y=88, hz=7.463, pitch=-8):
    '''Spawns sensor on a car. Input is a vehicle actor object'''
    if world==None:
    	client, world = util.link_server()

    #get camera blueprint
    blueprints = world.get_blueprint_library()
    cam_bp = blueprints.find('sensor.camera.semantic_segmentation')

    #set refresh rate, image size, and location on car
    if hz == 0:
        cam_bp.set_attribute('sensor_tick', str(0))
    else:
        cam_bp.set_attribute('sensor_tick', str(1.0/hz))
    cam_bp.set_attribute('image_size_x', str(img_x))
    cam_bp.set_attribute('image_size_y', str(img_y))
    relative_loc = carla.Location(x=2, z=1.4)
    relative_rot = carla.Rotation(pitch=pitch)
    cam_trans = carla.Transform(relative_loc,relative_rot)

    #spawn camera
    camera = world.spawn_actor(cam_bp, cam_trans, attach_to=car)
    print("Spawned semantic segmentation camera sensor")
    return camera

def spawn_3cam(car, img_x=200, img_y=88, yaw_deg=20, hz=0.2):
    '''
    spawn 3 cameras in same location on car with yaw offset
    yaw_deg is absolute angle between each side cam and car symmetry plane
    returns a list of 3 camera objects
    '''
    client, world = util.link_server()

    #get bp
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')

    #set refresh rate, size (same between all)
    cam_bp.set_attribute('sensor_tick', str(hz))
    cam_bp.set_attribute('image_size_x', str(img_x))
    cam_bp.set_attribute('image_size_y', str(img_y))

    #calculate rotated camera location
    center_cam_x = 2.2
    center_cam_y = 0
    center_cam_z = 1
    rotated_x, rotated_y = util.rotz(center_cam_x, center_cam_y, yaw_deg)

    #chassis coordinate system is right hand
    #carla world coord is left hand
    #spawn left cam
    relative_loc = carla.Location(x=rotated_x, y=-rotated_y, z=center_cam_z)
    relative_rot = carla.Rotation(yaw=-yaw_deg, pitch=-8)
    cam_trans = carla.Transform(relative_loc,relative_rot)
    left = world.spawn_actor(cam_bp, cam_trans, attach_to=car)

    #spawn center cam
    relative_loc = carla.Location(x=center_cam_x, z=center_cam_z)
    relative_rot = carla.Rotation(yaw=0, pitch=-8)
    cam_trans = carla.Transform(relative_loc,relative_rot)
    center = world.spawn_actor(cam_bp, cam_trans, attach_to=car)

    #spawn right cam
    relative_loc = carla.Location(x=rotated_x, y=rotated_y, z=center_cam_z)
    relative_rot = carla.Rotation(yaw=yaw_deg, pitch=-8)
    cam_trans = carla.Transform(relative_loc,relative_rot)
    right = world.spawn_actor(cam_bp, cam_trans, attach_to=car)

    return [left, center, right]

if __name__=="__main__":
    car, camera = simple_spawn('vehicle.tesla.*')
    pc = car.get_physics_control()
    for d in dir(pc):
        print(d, pc.__getattribute__(d))
