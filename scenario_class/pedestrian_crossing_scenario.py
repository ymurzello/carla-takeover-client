'''
bike crossing scenario class

Rocky Liang, 2020
'''
import carla

from carlahelp import util
from carlahelp.filehelp import read_json_config
from scenario_class.base_scenario import BaseScenario

class PedestrianCrossing(BaseScenario):
    def __init__(self):
        '''initialize crossing scenario class'''
        super().__init__()


    def load_config(self, path):
        '''read in config'''
        super().load_config(path)
        self.num_actors = len(self.config['pedestrians'])


    def spawn_npcs(self):
        '''
        spawn bike in the right location
        '''
        super().spawn_npcs()
        for idx, detail in self.config['pedestrians'].items():
            #spawn bike
            self._spawn_pedestrian(idx)


    def begin(self, control=None):
        '''call this to start moving the actors
        if no control argument is provided, it will read throttle for each individual pedestrian from config
        if provided, it will use that for all actors

        control: carla.VehicleControl object
        '''
        super().begin(control)
        self.enable_walker_ai()
        print ("Walkers AI enabled")

        


