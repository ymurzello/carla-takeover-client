'''
base scenario class

'''
import carla

from carlahelp import util
from carlahelp.filehelp import read_json_config

class BaseScenario:
    def __init__(self):
        '''initialize crossing scenario class'''
        self.config = None

        client, world = util.link_server()
        self.client = client
        self.world = world

        self.actors_list = []

        self.hero_id = None
        self._register_hero()

        self.pedestrian_target_list = list()
        self.pedestrian_list = list()


    def load_config(self, path):
        '''read in config'''
        self.config = read_json_config(path)


    def spawn_npcs(self):
        '''
        spawn bike in the right location
        '''
        return


    def get_distances(self):
        '''get event trigger distance'''
        dists = []
        if 'dist' in self.config:
            dist_dict = self.config['dist']
            for idx, d in dist_dict.items():
                dists.append(float(d))
        return dists


    def _spawn_bike(self, idx):
        '''helper function for spawning'''
        details = self.config['bikes'][idx]

        #get transform
        x = float(details['x'])
        y = float(details['y'])
        z = float(details['z'])
        yaw = float(details['yaw'])
        pitch = float(details['pitch'])
        roll = float(details['roll'])
        loc = carla.Location(x,y,z)
        rot = carla.Rotation(yaw=yaw, pitch=pitch, roll=roll)
        trans = carla.Transform(location=loc, rotation=rot)

        #spawn
        bp = self.world.get_blueprint_library().find(details['actor_type'])
        actor = self.world.try_spawn_actor(bp, trans)

        if actor!=None:
            self.actors_list.append(actor.id)
        else:
            self.world.debug.draw_point(loc, life_time=10)
            print("bike idx {} did not spawn, possibly due to collision".format(idx))
      

    def _spawn_vehicle(self, idx):
        details = self.config['vehicles'][idx]

        #get transform
        x = float(details['x'])
        y = float(details['y'])
        z = float(details['z'])
        yaw = float(details['yaw'])
        pitch = float(details['pitch'])
        roll = float(details['roll'])
        loc = carla.Location(x,y,z)
        rot = carla.Rotation(yaw=yaw, pitch=pitch, roll=roll)
        trans = carla.Transform(location=loc, rotation=rot)

        #spawn
        bp = self.world.get_blueprint_library().find(details['actor_type'])
        actor = self.world.try_spawn_actor(bp, trans)

        if actor!=None:
            self.actors_list.append(actor.id)
        else:
            self.world.debug.draw_point(loc, life_time=10)
            print("vehicle idx {} did not spawn, possibly due to collision".format(idx))


    def _spawn_pedestrian(self, idx):
        
        percentagePedestriansCrossing = 1.0     # how many pedestrians will walk through the road
        details = self.config['pedestrians'][idx]
        #get transform
        x = float(details['x'])
        y = float(details['y'])
        z = float(details['z'])
        yaw = float(details['yaw'])
        pitch = float(details['pitch'])
        roll = float(details['roll'])
        loc = carla.Location(x,y,z)
        rot = carla.Rotation(yaw=yaw, pitch=pitch, roll=roll)
        trans = carla.Transform(location=loc, rotation=rot)

        #spawn
        bp = self.world.get_blueprint_library().find(details['actor_type'])
        actor = self.world.try_spawn_actor(bp, trans)
        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)

        if actor!=None:
            self.actors_list.append(actor.id)

            tx = float(details['t_x'])
            ty = float(details['t_y'])
            tz = float(details['t_z'])
            speed = float(details['walker_speed'])
            target_location = carla.Location(x=tx, y=ty, z=tz)
            self.pedestrian_target_list.append((target_location, speed))
            self.pedestrian_list.append(actor.id)

        else:
            self.world.debug.draw_point(loc, life_time=10)
            print("bike idx {} did not spawn, possibly due to collision".format(idx))


    def enable_walker_ai(self):
        '''
        spawn and start autopilot for pedestrians
        '''
        self.ai_list = list()

        batch = []
        walker_ai_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for id, target in zip(self.pedestrian_list, self.pedestrian_target_list):
            walker = self.world.get_actor(id)
            target_transform = carla.Transform(target[0], carla.Rotation())
            batch.append(carla.command.SpawnActor(walker_ai_bp, target_transform, walker))
        results = self.client.apply_batch_sync(batch, True)
        for r in results:
            if r.error:
                print("walker control spawning error:", r.error)
            else:
                self.ai_list.append(r.actor_id)

        actor_list_walker_control = self.world.get_actors(self.ai_list)
        for i, a in enumerate(actor_list_walker_control):
            a.start()
            #if i>=5:
                #break



    def _register_hero(self):
        '''
        this class needs to know where the hero/ego car is
        registers hero car for scenario usage (i.e. distance based event triggers)
        if there are multiple hero cars, it will use the first one. called in init method
        '''
        actor_list = self.world.get_actors()
        while len(actor_list)==0:
            actor_list = self.world.get_actors()
        for a in actor_list:
            if a.attributes.get('role_name') == "hero":
                self.hero_id = a.id
                break
        if self.hero_id==None:
            print('No hero car registered')


    def check_distance(self, target=None):
        '''
        target: carla.Location object we want to check the distance to
        '''
        dist = 100000

        actors = self.world.get_actors(self.actors_list)
        if target==None:
            target = self.world.get_actor(self.hero_id).get_location()

        for actor in actors:
            di = util.distance(actor.get_location(), target)
            dist = min(dist, di)
        
        return dist


    def begin(self, control=None):
        '''call this to start moving the actors
        if no control argument is provided, it will read throttle for each individual bike from config
        if provided, it will use that for all actors

        control: carla.VehicleControl object
        '''
        return


    def stop(self):
        '''call this to let actors coast to a stop if still in motion'''
        control = carla.VehicleControl()
        self.client.apply_batch([carla.command.ApplyVehicleControl(x, control) for x in self.actors_list])


    def kill_npcs(self):
        '''remove all npcs spawned by this class'''
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actors_list])


