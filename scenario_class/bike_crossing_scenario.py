'''
bike crossing scenario class

'''
import carla

from carlahelp import util
from carlahelp.filehelp import read_json_config
from scenario_class.base_scenario import BaseScenario

class BikeCrossing(BaseScenario):
    def __init__(self):
        '''initialize crossing scenario class'''
        super().__init__()


    def load_config(self, path):
        '''read in config'''
        super().load_config(path)
        self.num_actors = len(self.config['bikes'])


    def spawn_npcs(self):
        '''
        spawn bike in the right location
        '''
        super().spawn_npcs()
        for idx, detail in self.config['bikes'].items():
            #spawn bike
            self._spawn_bike(idx)


    def begin(self, control=None):
        '''call this to start moving the actors
        if no control argument is provided, it will read throttle for each individual bike from config
        if provided, it will use that for all actors

        control: carla.VehicleControl object
        '''
        super().begin(control)

        if control==None:
            #control = carla.VehicleControl(self.config[''])
            controls = [carla.VehicleControl(throttle=float(self.config['bikes'][str(i)]['throttle'])) for i in range(self.num_actors)]
            self.client.apply_batch([carla.command.ApplyVehicleControl(x, control) for x, control in zip(self.actors_list, controls)])
            return

        self.client.apply_batch([carla.command.ApplyVehicleControl(x, control) for x in self.actors_list])


