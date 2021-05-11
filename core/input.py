'''
class that manages control input from steering wheel
'''
import sys
if sys.version_info >= (3, 0):
    from configparser import ConfigParser
else:
    from ConfigParser import RawConfigParser as ConfigPars

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
import carla
import math

class DualControl(object):
    def __init__(self, actor, world=None, start_in_autopilot=False, agent_controlled=False):
        '''
        Args:
        actor
        world
        start_in_autopilot
        agent_controlled - True, means that the actor is controlled by the BehaviourAgent, so this class no longer has to set actor.enable_autopilot().
                           False, means that the actor is controlled by the default autopilot. 
        '''
        self._agent_controlled = agent_controlled
        self._agent_autopilot_enabled = False # Toggled only when agent_controlled == True 
        self._autopilot_enabled = False # Toggled only when agent_controlled == False
        self._world = world

        if start_in_autopilot == True:
            if self._agent_controlled:
                self._agent_autopilot_enabled = True
            else:
                self._autopilot_enabled = True
        
        if isinstance(actor, carla.Vehicle):
            self._control = carla.VehicleControl()
            actor.set_autopilot(self._autopilot_enabled)
        elif isinstance(actor, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        #world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        try:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self._buttons = self._joystick.get_numbuttons()
            self._axes = self._joystick.get_numaxes()
            self._has_wheel = True
            self._js_name = self._joystick.get_name()
        except pygame.error:
            #print('No controller connected')
            self._has_wheel = False

        if self._has_wheel:
            self._parser = ConfigParser()
            self._parser.read('wheel_config.ini')
            self._steer_idx = int(
                self._parser.get(self._js_name, 'steering_wheel'))
            self._throttle_idx = int(
                self._parser.get(self._js_name, 'throttle'))
            self._brake_idx = int(self._parser.get(self._js_name, 'brake'))
            self._reverse_idx = int(self._parser.get(self._js_name, 'reverse'))
            self._handbrake_idx = int(
                self._parser.get(self._js_name, 'handbrake'))
            self._autopilot_idx = int(self._parser.get(self._js_name, 'autopilot'))
            self._leftturn_idx = int(self._parser.get(self._js_name, 'leftturn'))
            self._rightturn_idx = int(self._parser.get(self._js_name, 'rightturn'))
            self._leftlc_idx = int(self._parser.get(self._js_name, 'leftlc'))
            self._rightlc_idx = int(self._parser.get(self._js_name, 'rightlc'))

            if 'Logitech' in self._js_name:
                self._steer_gain = 1.0
            elif 'Sony' in self._js_name:
                self._steer_gain = 0.45
            else:
                self._steer_gain = 1.0

        self.hlc_state = 2

    def parse_events(self, actor, clock):
        '''
        this parses everything
        '''
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    #world.restart()
                    pass
                elif event.button == self._reverse_idx:
                    print('reverse')
                    self._control.gear = 1 if self._control.reverse else -1

            elif event.type == pygame.JOYBUTTONUP:
                if event.button == self._autopilot_idx:
                    if self._agent_controlled:
                        self._agent_autopilot_enabled = not self._agent_autopilot_enabled
                        print ('Agent autopilot toggled {}'.format(self._agent_autopilot_enabled))
                    else:
                        self._autopilot_enabled = not self._autopilot_enabled
                        print('autopilot toggled: {}'.format(self._autopilot_enabled))
                        actor.set_autopilot(self._autopilot_enabled)

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        #self._control.gear = world.player.get_control().gear
                        #world.hud.notification('%s Transmission' %
                                               #('Manual' if self._control.manual_gear_shift else 'Automatic'))
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        if self._agent_controlled:
                            self._agent_autopilot_enabled = not self._agent_autopilot_enabled
                            print ('Agent autopilot toggled {}'.format(self._agent_autopilot_enabled))
                        else:
                            self._autopilot_enabled = not self._autopilot_enabled
                            print('autopilot toggled: {}'.format(self._autopilot_enabled))
                            actor.set_autopilot(self._autopilot_enabled)
                            print('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                        # self._world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                    elif event.key == K_c:
                        print("Current transform: {}".format(self._world.player.get_transform()))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                if self._has_wheel:
                    self._parse_vehicle_wheel()
                    self._parse_wheel_keys()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            actor.apply_control(self._control)

        # added for disengage autopilot
        else:
            if isinstance(self._control, carla.VehicleControl):
                if self._has_wheel:
                    self._parse_vehicle_wheel_for_disengage_autopilot(actor)


        return self.hlc_state

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(self._axes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._buttons)]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1


        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd
        #print(brakeCmd, throttleCmd)

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])


    def _parse_vehicle_wheel_for_disengage_autopilot(self, actor):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(self._axes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._buttons)]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1


        #self._control.steer = steerCmd
        #self._control.brake = brakeCmd
        #self._control.throttle = throttleCmd
        #print(brakeCmd, throttleCmd)

        #toggle = jsButtons[self._reverse_idx]

        #self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

        # disable autopilot when steering brake or throttle input is detected
        if steerCmd > 0.004444 and self._autopilot_enabled:  # about 2 degrees of steering input
            if self._agent_controlled == True:
                self._agent_autopilot_enabled = False
                print('steering input triggers autopilot toggled: {}'.format(self._agent_autopilot_enabled))
            else:
                self._autopilot_enabled = False
                print('steering input triggers autopilot toggled: {}'.format(self._autopilot_enabled))
                actor.set_autopilot(self._autopilot_enabled)

        if brakeCmd > 0.10 and self._autopilot_enabled:  # about 10 percent of brake input
            if self._agent_controlled == True:
                self._agent_autopilot_enabled = False
                print('steering input triggers autopilot toggled: {}'.format(self._agent_autopilot_enabled))
            else:
                self._autopilot_enabled = False
                print('steering input triggers autopilot toggled: {}'.format(self._autopilot_enabled))
                actor.set_autopilot(self._autopilot_enabled)

        if throttleCmd > 0.10 and self._autopilot_enabled:  # about 10 percent of throttle input
            if self._agent_controlled == True:
                self._agent_autopilot_enabled = False
                print('steering input triggers autopilot toggled: {}'.format(self._agent_autopilot_enabled))
            else:
                self._autopilot_enabled = False
                print('steering input triggers autopilot toggled: {}'.format(self._autopilot_enabled))
                actor.set_autopilot(self._autopilot_enabled)





    def _parse_wheel_keys(self):
        '''
        used to check for continuous key presses on wheel
        '''
        button_presses = []
        for b in range(self._buttons):
            button = self._joystick.get_button(b)
            button_presses.append(button)
        if button_presses[self._rightlc_idx]:
            #r2, right lane change
            self.hlc_state = 5
        elif button_presses[self._leftlc_idx]:
            #l2, left lane change
            self.hlc_state = 6
        elif button_presses[self._rightturn_idx]:
            #r3, right turn
            self.hlc_state = 3
        elif button_presses[self._leftturn_idx]:
            #l3, left turn
            self.hlc_state = 4
        else:
            #default follow state
            self.hlc_state = 2

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
