'''
bike crossing scenario class

Rocky Liang, 2020
'''
import carla

from carlahelp import util
from carlahelp.filehelp import read_json_config

class BikeCrossing:
    def __init__(self):
        '''initialize crossing scenario class'''
        self.config = None

        client, world = util.link_server()
        self.client = client
        self.world = world

        self.bikes_list = []

        self.hero_id = None
        self._register_hero()

    def load_config(self, path):
        '''read in config'''
        self.config = read_json_config(path)
        self.num_bikes = len(self.config['bikes'])

    def spawn_npcs(self):
        '''
        spawn bike in the right location
        '''
        for idx, detail in self.config['bikes'].items():
            #spawn bike
            self._spawn_helper(idx)

    def get_distances(self):
        '''get event trigger distance'''
        dists = []
        dist_dict = self.config['dist']
        for idx, d in dist_dict.items():
            dists.append(float(d))
        return dists

    def _spawn_helper(self, idx):
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
            self.bikes_list.append(actor.id)
        else:
            self.world.debug.draw_point(loc, life_time=10)
            print("bike idx {} did not spawn, possibly due to collision".format(idx))

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
        bikes = self.world.get_actors(self.bikes_list)
        if target==None:
            target = self.world.get_actor(self.hero_id).get_location()
        dist = 100000

        for b in bikes:
            di = util.distance(b.get_location(), target)
            dist = min(dist, di)
        return dist

    def begin(self, control=None):
        '''call this to start moving the bikes
        if no control argument is provided, it will read throttle for each individual bike from config
        if provided, it will use that for all bikes

        control: carla.VehicleControl object
        '''
        if control==None:
            #control = carla.VehicleControl(self.config[''])
            controls = [carla.VehicleControl(throttle=float(self.config['bikes'][str(i)]['throttle'])) for i in range(self.num_bikes)]
            self.client.apply_batch([carla.command.ApplyVehicleControl(x, control) for x, control in zip(self.bikes_list, controls)])
            return

        self.client.apply_batch([carla.command.ApplyVehicleControl(x, control) for x in self.bikes_list])

    def stop(self):
        '''call this to let bikes coast to a stop if still in motion'''
        control = carla.VehicleControl()
        self.client.apply_batch([carla.command.ApplyVehicleControl(x, control) for x in self.bikes_list])


    def kill_npcs(self):
        '''remove all npcs spawned by this class'''
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.bikes_list])


if __name__=="__main__":

    bc = BikeCrossing()
    bc.load_config('scenario_configs/bike.json')
    bc.spawn_npcs()

    input("press enter")
    bc.kill_npcs()
