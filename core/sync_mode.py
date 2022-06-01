from collections import deque
import math
import os
try:
    import queue
except ImportError:
    import Queue as queue
import time

import carla

from carlahelp.filehelp import make_file_name, date_string, save_as_json, read_json_config
from carlahelp import spawn, util

class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, car, map, *sensors, **kwargs):
        self.world = world
        self.car = car
        self.map = map
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

        self.queue_len = kwargs.get('queue_len', 5)
        self.wp_spacing = kwargs.get('wp_spacing', 0.5)
        self.waypoint_queue = deque(maxlen=self.queue_len)
        #self.waypoint_queue = []
        self.waypoint = None

        self.actors = None
        self.vehicles_close = []
        self.vehicles_inrange = []
        self.pedestrians_close = []
        self.pedestrians_inrange = []

        self.record = kwargs.get('record', False)
        self.scenario = kwargs.get('scenario', False)

        self._saveid = 0
        if self.record==True:
            self._make_folders()
            self._hlc_count = {'follow':0, 'leftturn':0, 'rightturn':0, 'leftlc':0, 'rightlc':0}
            self._rgb_ext = '.png'
            self._ss_ext = '.png'

    def _make_folders(self):
        self._dicrectory_list = []
        #where all recording sessions are stored
        self.data_folder = 'Saved/'
        if not os.path.exists(self.data_folder):
            os.mkdir(self.data_folder)
        
        dstring = date_string(note=None)
        if not os.path.exists(self.data_folder):
            os.mkdir(self.data_folder)
        #where data from this session are stored
        self.session_folder = os.path.join(self.data_folder,dstring)
        if not os.path.exists(self.session_folder):
            os.mkdir(self.session_folder)

        targets_directory = os.path.join(self.session_folder, 'targets')
        os.mkdir(targets_directory)
        self._dicrectory_list.append(targets_directory)
        self._dicrectory_list.sort()
        print('Saving data to {}'.format(self.session_folder))

    # Added by Sandy to retrieve session folder
        global working_folder
        working_folder = dstring

    def working_folder(self):
        global working_folder
        return working_folder 

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def set_record(self, record=False):
        self.record = record

    def tick(self, timeout, hlc):
        self.frame = self.world.tick()
        self._update_actors()
        self._populate_queue(hlc)
        self._get_other_vehicles()
        self._get_other_pedestrians()

        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data

    def _update_actors(self):
        self.actors = self.world.get_actors()
        #for a in self.actors.filter('walker.*'):
            #print(a.type_id)

    def _get_other_vehicles(self):
        vehicles = self.actors.filter('vehicle.*')
        t = self.car.get_transform()
        self.vehicles_close.clear()
        self.vehicles_inrange.clear()
        if len(vehicles) > 1:
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles_all = [(distance(x.get_location()), x) for x in vehicles if x.id != self.car.id]

            for d, vehicle in sorted(vehicles_all):
                self.vehicles_inrange.append((d, vehicle))
                if d > 54.5:
                    break
                #cars within 30 meters
                delta_x, delta_y = util.car_frame_deltas(t, vehicle.get_location())
                heading_delta = vehicle.get_transform().rotation.yaw - t.rotation.yaw
                #print(abs(heading_delta), str(vehicle))
                if delta_x > 0 and abs(heading_delta) < 90 and abs(delta_y) < 3.0:
                    #cars in front and within a certain lateral distance
                    self.vehicles_close.append((d, vehicle))
                    #print(vehicle.__str__(), delta_y)

    def _get_other_pedestrians(self):
        pedestrians = self.actors.filter('walker.*')
        #print(len(pedestrians))
        t = self.car.get_transform()
        self.pedestrians_close.clear()
        self.pedestrians_inrange.clear()
        if len(pedestrians) > 0:
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            pedestrians_all = [(distance(x.get_location()), x) for x in pedestrians]
            for d, walker in sorted(pedestrians_all, key=lambda tup: tup[0]):
                self.pedestrians_inrange.append((d, walker))
                if d > 54.5:
                    break
                delta_x, delta_y = util.car_frame_deltas(t, walker.get_location())
                heading_delta = walker.get_transform().rotation.yaw - t.rotation.yaw
                if delta_x > 0 and abs(delta_y) < 3.0:
                    #cars in front and within a certain lateral distance
                    self.pedestrians_close.append((d, walker))
                    #print(vehicle.__str__(), delta_y)

    def _populate_queue(self, hlc):
        issued_hlc = hlc

        #print(len(self.waypoint_queue))
        trans = self.car.get_transform()
        loc = trans.location
        closest_wp = self.map.get_waypoint(loc)

        #remove close waypoints from buffer
        push_ind = -1
        for i, wp in enumerate(self.waypoint_queue):
            dist_to_car = util.distance(wp.transform.location, loc)
            if dist_to_car < 1.5:
                push_ind = i

        if push_ind >= 0:
            for i in range(push_ind+1):
                #print('pop')
                self.waypoint_queue.popleft()


        if self.waypoint_queue:

            if (not closest_wp.is_junction):
                #distance = util.distance(self.waypoint_queue[0].transform.location,loc)
                delta_x, delta_y = util.car_frame_deltas(trans, self.waypoint_queue[0].transform.location)
                if delta_y < 0.0:
                    #print('CLEARING behind\n')
                    self.waypoint_queue.clear()
                else:
                    dist = math.sqrt((delta_x**2) + (delta_y**2))
                    #print(dist)
                    #set threshold to more than waypoint separation
                    if dist > 1.5:
                        #print('CLEARING too far\n')
                        self.waypoint_queue.clear()


        lane_change_flag = False
        while len(self.waypoint_queue) < self.queue_len:
            '''
            in here if queue needs more waypoints
            each iteration of while loop adds one wp
            lane splits dont neccesarily happen at a waypoint with "is_junction==True".
            use waypoint.lane_change to filter for points where lc can be done
            use waypoint.lane_type to filter for drivable lanes
            DISABLE LANE CHANGE WHEN LC WP HAS A LARGE ENOUGH HEADING DIFFERENCE (OPPOSITE)
            '''

            if self.waypoint_queue:
                #not empty
                front_wp = self.waypoint_queue[-1]

                '''lane change logic'''
                if hlc == 5 and (not closest_wp.is_junction):
                    if lane_change_flag==False:
                        if str(front_wp.lane_change) == 'Both' or str(front_wp.lane_change == 'Right'):
                            #if there is a lane change available, get it
                            next = front_wp.get_right_lane()
                            if (next != None):
                                next_yaw_diff = abs(front_wp.transform.rotation.yaw-next.transform.rotation.yaw)
                                if str(next.lane_type) == 'Driving' and next_yaw_diff<90:
                                    lane_change_flag = True
                                else:
                                    hlc = 2
                                    lane_change_flag = False
                        else:
                            hlc = 2
                    else:
                        hlc = 2

                elif hlc == 6 and (not closest_wp.is_junction):
                    if lane_change_flag==False:
                        if str(front_wp.lane_change) == 'Both' or str(front_wp.lane_change == 'Left'):
                            next = front_wp.get_left_lane()
                            if (next != None):
                                next_yaw_diff = abs(front_wp.transform.rotation.yaw-next.transform.rotation.yaw)
                                if str(next.lane_type) == 'Driving' and next_yaw_diff<90:
                                    lane_change_flag = True
                                else:
                                    hlc = 2
                                    lane_change_flag = False
                        else:
                            hlc = 2
                    else:
                        hlc = 2
                else:
                    if issued_hlc >= 2 and issued_hlc <=4:
                        hlc = issued_hlc
                    else:
                        hlc = 2

                '''follow/turn logic'''
                if (hlc >= 2) and (hlc <= 4):
                    #not lane change
                    #print(front_wp.is_junction, '{}'.format(len(front_wp.next(self.wp_spacing))))
                    next_list = front_wp.next(self.wp_spacing)
                    if len(next_list) == 1:
                        #no available turns
                        next = next_list[0]
                    else:
                        #turn selection
                        #loop thru wp list
                        smallest_delta = 180
                        least_delta_idx = 0
                        largest = 0
                        largest_idx = 0
                        smallest = 180
                        smallest_idx = 0
                        #print("deltas:")
                        for idx, w in enumerate(next_list):
                            delta = util.angle_wrap(w.transform.rotation.yaw - front_wp.transform.rotation.yaw)
                            delta_abs = abs(delta)
                            #print(delta)
                            if delta_abs < smallest_delta:
                                smallest_delta = delta_abs
                                least_delta_idx = idx
                            if delta < smallest:
                                smallest = delta
                                smallest_idx = idx
                            if delta > largest:
                                largest = delta
                                largest_idx = idx
                        #print("index:",least_delta_idx,'\n')
                        try:
                            #print('hlc:', hlc)
                            if hlc==2:
                                #straight
                                #print("follow index:",least_delta_idx,'\n')
                                next = next_list[least_delta_idx]
                            elif hlc==3:
                                #right
                                #print("largest index:",largest_idx,'\n')
                                next = next_list[largest_idx]
                            elif hlc==4:
                                #left
                                #print("smallest index:",smallest_idx,'\n')
                                next = next_list[smallest_idx]
                        except IndexError:
                            next = self.map.get_waypoint(loc)



                assert next != None
                self.waypoint_queue.append(next)

            else:
                #empty queue
                #print('empty')
                wp = self.map.get_waypoint(loc)
                self.waypoint_queue.append(wp)


        #draw_path(self.waypoint_queue, self.world, time=0.05, z=2, max_num = 3)

    def record_frame(self, snapshot, transform, velocity, control, affordance, wp, hlc, stage, collision_flag, lane_invaded_flag, travel):
        '''
        called once per loop to record everything
        '''
        frame = {}
        #time
        frame['gametime hh:mm:ss'] = str(time.strftime("%H:%M:%S", time.gmtime(snapshot.timestamp.elapsed_seconds)))
        frame['gametime ss.ms'] = str(snapshot.timestamp.elapsed_seconds)
        frame['time_stamp'] = str(time.time())

        #vehicle states
        ang = self.car.get_angular_velocity()
        acc = self.car.get_acceleration()

        vx, vy = util.measure_forward_velocity(velocity, transform.rotation, return_both=True)
        vx_kph = vx*3.6
        frame['car_vx'] = str(vx)
        frame['car_vx_kph'] = str(vx_kph)
        frame['car_vy'] = str(vy)
        frame['car_vz'] = str(velocity.z)
        frame['global_ax'] = str(acc.x)
        frame['global_ay'] = str(acc.y)
        frame['global_az'] = str(acc.z)
        frame['global_rollrate'] = str(ang.x)
        frame['global_pitchrate'] = str(ang.y)
        frame['global_yawrate'] = str(ang.z)
        frame['global_x'] = str(transform.location.x)
        frame['global_y'] = str(transform.location.z)
        frame['global_z'] = str(transform.location.y)
        frame['global_heading'] = str(transform.rotation.yaw)

        #distance travelled
        d_travel = travel
        frame['distance_travelled'] = str(d_travel)

        #control states
        frame['throttle'] = str(control.throttle)
        frame['steer'] = str(control.steer)
        frame['brake'] = str(control.brake)
        frame['handbrake'] = str(int(control.hand_brake))
        frame['reverse'] = str(int(control.reverse))
        frame['gear'] = str(control.gear)

        assert isinstance(hlc, int)
        frame['hlc'] = str(hlc)
        frame['stage'] = str(stage)

        #road info
        frame['headingerror'] = str(affordance[0])
        frame['cte'] = str(affordance[1])
        frame['k'] = str(affordance[2])
        frame['dist_to_car'] = str(affordance[3])
        frame['dist_to_walker'] = str(affordance[4])
        frame['is_junction'] = str(wp.is_junction)
        frame['lane_id'] = str(wp.id)

        #lane change
        lane_invaded = lane_invaded_flag
        frame['lane_change'] = str(lane_invaded)
        #print(str(lane_invaded))

        #collision
        col_flag = collision_flag
        frame['collision'] = str(col_flag)
        #print(str(col_flag))

        #other cars
        frame['nearby_cars'] = {}
        for n, npc_car in enumerate(self.vehicles_inrange):
            dist = npc_car[0]
            npc_car = npc_car[1]

            npc_car_trans = npc_car.get_transform()
            frame['nearby_cars'][str(n)] = {}
            frame['nearby_cars'][str(n)]['type_id'] = npc_car.type_id
            frame['nearby_cars'][str(n)]['global_x'] = str(npc_car_trans.location.x)
            frame['nearby_cars'][str(n)]['global_y'] = str(npc_car_trans.location.y)
            frame['nearby_cars'][str(n)]['global_z'] = str(npc_car_trans.location.z)
            frame['nearby_cars'][str(n)]['global_heading'] = str(npc_car_trans.rotation.yaw)
            frame['nearby_cars'][str(n)]['distance'] = str(dist)

        #other walkers
        frame['nearby_walkers'] = {}
        for n, npc_walk in enumerate(self.pedestrians_inrange):
            dist = npc_walk[0]
            npc_walk = npc_walk[1]

            npc_walk_trans = npc_walk.get_transform()
            frame['nearby_walkers'][str(n)] = {}
            frame['nearby_walkers'][str(n)]['type_id'] = npc_walk.type_id
            frame['nearby_walkers'][str(n)]['global_x'] = str(npc_walk_trans.location.x)
            frame['nearby_walkers'][str(n)]['global_y'] = str(npc_walk_trans.location.y)
            frame['nearby_walkers'][str(n)]['global_z'] = str(npc_walk_trans.location.z)
            frame['nearby_walkers'][str(n)]['global_heading'] = str(npc_walk_trans.rotation.yaw)
            frame['nearby_walkers'][str(n)]['distance'] = str(dist)


        jname = make_file_name(self._saveid, prefix='target_', nlen=6, ext='.json')
        save_as_json(os.path.join(self.session_folder, 'targets', jname), frame)
        self._saveid += 1
        if hlc == 2:
            self._hlc_count['follow'] += 1
        elif hlc == 3:
            self._hlc_count['rightturn'] += 1
        elif hlc == 4:
            self._hlc_count['leftturn'] += 1
        elif hlc == 5:
            self._hlc_count['rightlc'] += 1
        elif hlc == 6:
            self._hlc_count['leftlc'] += 1
