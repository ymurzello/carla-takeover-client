#testing for getting info from sensors
import carla
import util
import spawn
import remove
import math
import numpy as np

import pygame
pygame.init()

#make window and title
DISPLAY_W = 1280
DISPLAY_H = 720
FPS = 30
win = pygame.display.set_mode((DISPLAY_W,DISPLAY_H))
pygame.display.set_caption('First Game')
clock = pygame.time.Clock()

#see if car/camera already exists on server
client, world = util.link_server()
carlist = world.get_actors().filter('vehicle.*')
if (len(carlist)!=0):
    car = carlist[0]
    camera = world.get_actors().filter('sensor.camera.*')[0]
else:
    #spawn new
    car, camera = spawn.simple_spawn()

def get_rgb(image):
    #print('handling pic{}'.format(image.frame_number))
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (DISPLAY_H,DISPLAY_W,4))
    array = array[:,:,:3]
    array = array[:,:,::-1]
    surface = pygame.surfarray.make_surface(array.swapaxes(0,1))
    win.blit(surface,(0,0))

camera.listen(lambda image: get_rgb(image))

car.set_autopilot(True)
run = True
while run:
    #handle closing the window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        run = False

    pygame.display.flip()
    #tell the game to run in 60 fps
    clock.tick_busy_loop(FPS)

pygame.quit()
quit()
