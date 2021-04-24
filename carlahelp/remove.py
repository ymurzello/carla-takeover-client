#functions for removing actors

try:
    from carlahelp import util
except ModuleNotFoundError:
    import util

def simple_kill():
    #simple removal of ONE earliest spawned actor and sensor
    client, world = util.link_server()
    #get list of vehicle and sensor actors
    carlist = world.get_actors().filter('vehicle.*')
    sensorlist = world.get_actors().filter('sensor.*')

    if(len(carlist)!=0):
        #if there is at least one car
        print('cars to be removed:')
        print(carlist[0])
        carlist[0].destroy()
    else:
        print('no cars found on server')
    print('')

    if(len(sensorlist)!=0):
        #if there is at least one sensor
        print('sensors to be removed:')
        print(sensorlist[0])
        sensorlist[0].destroy()
    else:
        print('no sensors found on server')
    print('')


if __name__=="__main__":
    simple_kill()
