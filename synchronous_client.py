#!/usr/bin/env python

'''
client for running scenarios

Rocky Liang, 2020
'''

#USE carla99 env

import glob
import os
import sys

import argparse
import math

from carlahelp import spawn, util
from carlahelp.filehelp import make_file_name, date_string, save_as_json, read_json_config
from core.input import DualControl
from core.sync_mode import CarlaSyncMode
from bike_crossing import BikeCrossing

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass

import carla

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

MIRROR_W = 350
MIRROR_H = 200

DISPLAY_W = 1080
DISPLAY_H = 720

SCREEN_W = MIRROR_W*2+DISPLAY_W
SCREEN_H = max(DISPLAY_H, MIRROR_H)

FRONT_W = 300
FRONT_H = 60

FREQ = 15

def image_np(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    return array[:, :, ::-1]

def draw_image(surface, image, blend=False):
    array = image_np(image)
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (MIRROR_W, 0))

def draw_cam(surface, image, u, v):
    array = image_np(image)
    array = np.fliplr(array)
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    #surface.blit(image_surface,(0,DISPLAY_H-FRONT_H))
    surface.blit(image_surface,(u,v))

def get_font(font_size=14):
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, font_size)

def hlc_string(hlc):
    if hlc == 2:
        return "Follow"
    elif hlc == 3:
        return "Right Turn"
    elif hlc == 4:
        return "Left Turn"
    elif hlc == 5:
        return "Right Lane Change"
    else:
        return "Left Lane Change"

def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def main(args):
    actor_list = []
    sensor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (SCREEN_W, SCREEN_H),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    fill_rect = pygame.Rect((0,0,MIRROR_W, SCREEN_H/2))
    pygame.display.set_caption('CARLA Scenarios')
    font = get_font()
    font_big = get_font(font_size=30)
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()
    world = client.reload_world()

    config = read_json_config(args.spawn_config)

    try:
        m = world.get_map()

        sx, sy, sz = float(config['ego_actor']['x']), float(config['ego_actor']['y']), float(config['ego_actor']['z'])
        syaw, spitch, sroll = float(config['ego_actor']['yaw']), float(config['ego_actor']['pitch']), float(config['ego_actor']['roll'])
        s_loc = carla.Location(sx, sy, sz+1)
        s_rot = carla.Rotation(yaw=syaw, pitch=spitch, roll=sroll)
        start_pose = carla.Transform(location=s_loc, rotation=s_rot)

        #spawn car
        blueprint_library = world.get_blueprint_library()
        car_bp = blueprint_library.find(config['ego_actor']['actor_type'])

        #car_bp.set_attribute('color','255,20,147')
        #car_bp.set_attribute('color','158,255,0')
        car_bp.set_attribute('role_name', 'hero')
        vehicle = world.spawn_actor(
            car_bp,
            start_pose)
        #print(vehicle.attributes.get('role_name'))
        actor_list.append(vehicle)

        #spawn spectator cam
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(DISPLAY_W))
        cam_bp.set_attribute('image_size_y', str(DISPLAY_H))
        #cam_bp.set_attribute('sensor_tick',str(FREQ))
        camera_rgb = world.spawn_actor(
            cam_bp,
            #carla.Transform(carla.Location(x=-5.2, z=3.3), carla.Rotation(pitch=10)),
            carla.Transform(carla.Location(x=1, z=1), carla.Rotation(yaw=180, pitch=-50)),
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.SpringArm)

        #every sensor must be registered by
        #appending to actor list AND sensor list
        actor_list.append(camera_rgb)
        sensor_list.append(camera_rgb)

        #rearview cam
        #camera_front = spawn.spawn_rgb(vehicle, world, img_x=FRONT_W, img_y=FRONT_H, hz=FREQ, pitch=-12)
        cam_bp.set_attribute('image_size_x', str(FRONT_W))
        cam_bp.set_attribute('image_size_y', str(FRONT_H))
        cam_bp.set_attribute('fov', str(60))
        mirror_rear = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=-2, z=1.3), carla.Rotation(yaw=-180)),
            attach_to=vehicle)
        actor_list.append(mirror_rear)
        sensor_list.append(mirror_rear)

        #spawn left mirror
        cam_bp.set_attribute('image_size_x', str(MIRROR_W))
        cam_bp.set_attribute('image_size_y', str(MIRROR_H))
        cam_bp.set_attribute('fov', str(70))

        mirror_left = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(y=-1.1, z=1.0), carla.Rotation(yaw=-160)),
            attach_to=vehicle)

        #every sensor must be registered by
        #appending to actor list AND sensor list
        actor_list.append(mirror_left)
        sensor_list.append(mirror_left)

        #spawn right mirror
        cam_bp.set_attribute('image_size_x', str(MIRROR_W))
        cam_bp.set_attribute('image_size_y', str(MIRROR_H))
        cam_bp.set_attribute('fov', str(70))
        mirror_right = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(y=1.1, z=1.0), carla.Rotation(yaw=160)),
            attach_to=vehicle)

        #every sensor must be registered by
        #appending to actor list AND sensor list
        actor_list.append(mirror_right)
        sensor_list.append(mirror_right)


        controller = DualControl(vehicle, world=world, start_in_autopilot=False)

        if args.scenario:
            bike_crossing = BikeCrossing()
            bike_crossing.load_config('scenario_configs/bike.json')
            bike_crossing.spawn_npcs()
            trigger_distances = bike_crossing.get_distances()
            beep = pygame.mixer.Sound('assets/sounds/bell.wav')
            wheel_icon = pygame.image.load('assets/icons/red.gif').convert_alpha()
            sound_time = 0
            flash_time = 0

        #initial scenario stage
        flash_on = False
        stage = 0

        #initial hlc
        hlc = 2


        # Create a synchronous mode context.
        #SENSORS SHOULD BE PASSED IN THE SAME ORDER AS IN ACTOR_LIST
        with CarlaSyncMode(world, vehicle, m, *sensor_list, fps=FREQ, record=args.record, scenario=args.scenario) as sync_mode:

            record_start_flag = False
            yaw_prev = None
            while True:
                if should_quit():
                    return

                #print(sync_mode._queues)
                clock.tick()

                # Advance the simulation and wait for the data.
                tick_data = sync_mode.tick(timeout=2.0, hlc=hlc)
                snapshot = tick_data[0]
                image_rgb = tick_data[1]

                image_front = tick_data[2]
                image_mirror_left = tick_data[3]
                image_mirror_right = tick_data[4]


                #parse for control input and run vehicle
                hlc = controller.parse_events(vehicle, clock)
                control_states = vehicle.get_control()

                #draw trail based on hlc
                #sync_mode.draw_trail(hlc)

                v = vehicle.get_velocity()
                trans = vehicle.get_transform()
                #print('{:.3f}, {:.3f}'.format(trans.location.x, trans.location.y))


                closest_wp = m.get_waypoint(trans.location)
                draw_loc = closest_wp.transform.location+carla.Location(z=2)
                #sync_mode.world.debug.draw_point(draw_loc, life_time=0.05)
                #print(closest_wp.is_junction, len(closest_wp.next(0.5)))

                #track SECOND waypoint in queue as first does not shift with lc command
                second_wp = sync_mode.waypoint_queue[1]
                heading_error = second_wp.transform.rotation.yaw - trans.rotation.yaw
                heading_error = util.angle_wrap(heading_error)
                delta_x, delta_y = util.car_frame_deltas(trans, second_wp.transform.location)

                vx, vy = util.measure_forward_velocity(v, trans.rotation, return_both=True)
                vx_kph = vx*3.6
                curvature = util.curvature(sync_mode.waypoint_queue[1].transform, sync_mode.waypoint_queue[2].transform)

                #lateral accel
                #r = 1/(curvature + 1e-6)
                #print(vx**2/r)

                if len(sync_mode.vehicles_close)>0:
                    #print(sync_mode.vehicles_close[0][0], sync_mode.vehicles_close[0][1].__str__())
                    dist_to_car = max(sync_mode.vehicles_close[0][0] - 4.5, 0)
                else:
                    dist_to_car = 50.0

                if len(sync_mode.pedestrians_close)>0:
                    #print(sync_mode.vehicles_close[0][0], sync_mode.vehicles_close[0][1].__str__())
                    dist_to_walker = max(sync_mode.pedestrians_close[0][0] - 4.5, 0)
                else:
                    dist_to_walker = 50.0

                affordance = (heading_error, delta_y, curvature, dist_to_car, dist_to_walker)



                '''
                scenario logic
                '''
                if sync_mode.scenario:
                    #check distance to bike
                    bike_dist = bike_crossing.check_distance(trans.location)
                    #if distance is below threshold, do something
                    #first stage: flash warning
                    #2nd stage: bike starts crossing

                    if stage==0:
                        if bike_dist<trigger_distances[0]:
                            #stage 0 to 1 or 0 to 3 transition
                            if controller._autopilot_enabled:
                                stage = 1
                            else:
                                stage = 3
                    elif stage==1:
                        #stage 1: play warning sound
                        if snapshot.timestamp.elapsed_seconds - sound_time > 3:
                            beep.play()
                            sound_time = snapshot.timestamp.elapsed_seconds
                        if snapshot.timestamp.elapsed_seconds - flash_time > 1.5:
                            flash_on = not flash_on
                            flash_time = snapshot.timestamp.elapsed_seconds

                        if bike_dist<trigger_distances[1]:
                            flash_on = False
                            #stage one to two transition
                            bike_crossing.begin()
                            #disable autopilot
                            controller._autopilot_enabled = False
                            print('autopilot toggled: {}'.format(controller._autopilot_enabled))
                            sync_mode.car.set_autopilot(controller._autopilot_enabled)
                            stage = 2
                        elif controller._autopilot_enabled == False:
                            #stage 1 to 3 transition
                            flash_on = False
                            stage = 3
                    elif stage==2:
                        #delta, throttle, brake = driver.drive(heading_error, delta_y, vx, curvature, 28, min(dist_to_car, dist_to_walker))
                        vc = carla.VehicleControl(throttle=0, steer=0, brake=0.02)
                        sync_mode.car.apply_control(vc)
                    elif stage == 3:
                        #stage 3, bike crossing still gets triggered
                        #but driver stays in control
                        if bike_dist<trigger_distances[1]:
                            bike_crossing.begin(carla.VehicleControl(throttle=0.3))
                            stage = 4
                        elif controller._autopilot_enabled:
                            stage = 1

                    elif stage==4:
                        #nothing happens in stage 4
                        pass

                '''end of scenario code'''

                #record
                if vx > 0.01:
                    record_start_flag = True
                if sync_mode.record and record_start_flag:
                    #sync_mode.record_image(tick_data[1:])
                    sync_mode.record_frame(snapshot, trans, v, control_states, affordance, second_wp, hlc, stage)

                #image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # Draw the display.
                display.fill((0,0,0), rect=fill_rect)
                draw_image(display, image_rgb)
                draw_cam(display, image_front, (SCREEN_W/2)-FRONT_W/2, 0)
                draw_cam(display, image_mirror_left, 0, SCREEN_H-MIRROR_H)
                draw_cam(display, image_mirror_right, MIRROR_W+DISPLAY_W, SCREEN_H-MIRROR_H)

                if flash_on:
                    display.blit(
                        font_big.render('Please takeover control of vehicle', True, (255,0,0)),
                        (SCREEN_W//2-265, SCREEN_H//2))
                    display.blit(wheel_icon, (SCREEN_W//2-64,SCREEN_H//2 + 64))


                display.blit(
                    font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                    (8, 28))
                display.blit(
                    font.render('steer: {0:2.2f}, brake: {1:2.2f}, gas: {2:2.2f}'.format(control_states.steer,control_states.brake,control_states.throttle), False, (255, 255, 255)),
                    (8, 46))
                display.blit(
                    font.render('% 5d Gear' % control_states.gear, False, (255, 255, 255)),
                    (8, 64))
                hlc_text = font_big.render('% s' % hlc_string(hlc), False, (255, 99, 71))
                hlc_width = hlc_text.get_rect().width
                display.blit(
                    font.render('% 5d Heading Error [deg]' % heading_error, False, (255, 255, 255)),
                    (8, 118))
                display.blit(
                    font.render('% .2f CTE [m]' % delta_y, False, (255, 255, 255)),
                    (8, 136))
                display.blit(
                    font.render('% .2f Curvature' % curvature, False, (255, 255, 255)),
                    (8, 154))
                display.blit(
                    font.render('% .2f Dist to car' % dist_to_car, False, (255, 255, 255)),
                    (8, 172))
                display.blit(
                    font.render('% .2f Dist to pedestrian' % dist_to_walker, False, (255, 255, 255)),
                    (8, 190))

                display.blit(
                    font_big.render('% 5d' % vx_kph, False, (0, 255, 255)),
                    ((SCREEN_W/2)-60, SCREEN_H-50))
                pygame.display.flip()

    finally:

        print('destroying actors.')
        if args.scenario:
            bike_crossing.kill_npcs()
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--record', type=bool, default=False)
    parser.add_argument('-s', '--scenario', type=bool, default=False)
    parser.add_argument('-sp', '--spawn_config', type=str, default='spawn_configs/test2.json')
    parser.add_argument('-sc', '--scenario_config', type=str, default='scenario_configs/bike.json')
    args = parser.parse_args()

    try:

        main(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')