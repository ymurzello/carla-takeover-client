'''
bike crossing scenario class

Rocky Liang, 2020
'''
import carla

from carlahelp import util
from carlahelp.filehelp import read_json_config
from scenario_class.base_scenario import BaseScenario

class CarCrashScenario(BaseScenario):
    def __init__(self):
        '''initialize crossing scenario class'''
        super().__init__()


    def load_config(self, path):
        '''read in config'''
        super().load_config(path)
        self.num_actors = len(self.config['vehicles'])


    def spawn_npcs(self):
        '''
        spawn vehicle in the right location
        '''
        super().spawn_npcs()
        for idx, detail in self.config['vehicles'].items():
            #spawn vehicles
            self._spawn_vehicle(idx)



