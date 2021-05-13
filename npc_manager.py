'''
class that spawns and keeps track of CARLA NPCs

Rocky Liang, 2020
'''
import argparse

import carla

from carlahelp import util
from carlahelp.filehelp import read_json_config


class NPCManager:
    def __init__(self):
        '''
        initialize class
        '''
        self.config = None

        client, world = util.link_server()
        self.client = client
        self.world = world
        self.blueprint_library = world.get_blueprint_library()

        self.tm = client.get_trafficmanager()
        self.tm.set_hybrid_physics_mode(True)
        self.tm.set_global_distance_to_leading_vehicle(1)
        #self.tm.global_percentage_speed_difference(100)
        #for m in dir(self.tm):
            #print(m)

        #actor id containers
        self.cars_list = []
        self.peds_list = []
        self.ai_list = []
        self.target_list = []

    def load_config(self, path):
        '''read json config file'''
        self.config = read_json_config(path)

    def spawn_npcs(self):
        '''run at beginning of scenario to spawn
        all npcs designated in config

        the config has to be loaded before calling this
        '''
        #check config is loaded
        if not self.config:
            print("Spawn configuration has not been loaded, \ncall load_config method first")
            return

        #spawn npc (look at recipe)
        for idx, car_detail in self.config['cars'].items():
            #call vehicle spawn method here
            self._spawn_helper(idx, 'vehicle.*')
            #pass

        for idx, ped_detail in self.config['walkers'].items():
            #call pedestrian spawn method here
            self._spawn_helper(idx, 'walker.pedestrian.*')
            #pass

    def _spawn_helper(self, idx, actor_pattern):
        '''
        spawns one car or pedestrian

        idx: key for vehicle information in self.config dict
        actor_pattern: glob pattern for which type of actor is being spawned
        '''
        if actor_pattern=="vehicle.*":
            details = self.config['cars'][idx]
        elif actor_pattern=="walker.pedestrian.*":
            details = self.config['walkers'][idx]

        x = float(details['x'])
        y = float(details['y'])
        #add to height so npc doesn't interfere with the ground
        z = float(details['z']) + 2
        yaw = float(details['yaw'])
        pitch = float(details['pitch'])
        roll = float(details['roll'])
        loc = carla.Location(x=x, y=y, z=z)
        rot = carla.Rotation(yaw=yaw, pitch=pitch, roll=roll)
        trans = carla.Transform(location=loc, rotation=rot)

        #get blueprint
        bp = self.blueprint_library.find(details['actor_type'])
        actor = self.world.try_spawn_actor(bp, trans)

        if actor != None:
            if actor_pattern=='vehicle.*':
                self.cars_list.append(actor.id)
            elif actor_pattern=='walker.pedestrian.*':
                #walker_ai = self.world.spawn_actor(walker_ai_bp, carla.Transform(), actor)
                tx = float(details['t_x'])
                ty = float(details['t_y'])
                tz = float(details['t_z'])
                speed = float(details['walker_speed'])
                target_location = carla.Location(x=tx, y=ty, z=tz)
                #self.ai_list.append(walker_ai.id)
                self.target_list.append((target_location, speed))
                self.peds_list.append(actor.id)
        else:
            self.world.debug.draw_point(loc, life_time=10)
            print("actor idx {} of type {} did not spawn, possibly due to collision".format(idx, actor_pattern))

    def enable_autopilot(self, enabled=True):
        '''autopilot for all vehicles (not pedestrians), does not affect ego vehicle
        enabled: bool for whether you want autopilot to be on for all npc cars
        '''
        tm_port = self.tm.get_port()
        car_actors = self.world.get_actors(self.cars_list)
        for a in car_actors:
            a.set_autopilot(enabled, tm_port)

    def enable_walker_ai(self):
        '''
        spawn and start autopilot for pedestrians
        '''
        batch = []
        walker_ai_bp = self.blueprint_library.find('controller.ai.walker')
        for id, target in zip(self.peds_list, self.target_list):
            walker = self.world.get_actor(id)
            batch.append(carla.command.SpawnActor(walker_ai_bp, carla.Transform(), walker))
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

    def kill_npcs(self):
        '''call at the end of scenario to remove all npcs
        spawned by the manager
        '''
        actor_list = self.world.get_actors()
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.cars_list])
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.peds_list])
        for a in actor_list.filter('controller.*'):
            a.stop()
            a.destroy()


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-sp', '--spawn-config', type=str, default='spawn_configs/test2.json')
    args = parser.parse_args()
    try:
        manager = NPCManager()
        manager.load_config(args.spawn_config)
        manager.spawn_npcs()

        manager.enable_autopilot()
        manager.enable_walker_ai()

        input('press enter to exit ')
    finally:
        manager.kill_npcs()
