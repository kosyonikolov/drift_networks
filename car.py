import copy

import tyre
import util


class Car:
    g = 9.8

    def __init__(self, engine, max_brake_force, width, height, mass, tyre_type: tyre.Tyre):
        self.engine = engine
        self.max_brake_force = max_brake_force

        self.width = width
        self.height = height

        self.steering = 0
        self.mass = mass

        self.vx = 0
        self.vy = 0

        self.fl_tyre = copy.deepcopy(tyre_type)
        self.fr_tyre = copy.deepcopy(tyre_type)
        self.rl_tyre = copy.deepcopy(tyre_type)
        self.rr_tyre = copy.deepcopy(tyre_type)

    # get_velocity is blah blah blah
    def get_velocity(self, dt, throttle_input, brake_input, steering_input):
        self.steering = steering_input

        # assumption for a perfect distribution
        distributed_load = self.mass * Car.g * 0.25

        vx_left, vy_left = util.rotate_2d(self.vx, self.vy, self.steering)
        vx_right, vy_right = util.rotate_2d(self.vx, self.vy, self.steering)

        rpm = self.vy / self.fr_tyre.get_radius()

        engine_force = self.engine.get_force(rpm, throttle_input)

        brake_force = - brake_input * self.max_brake_force

        fl_x, fl_y = self.fr_tyre.get_force(distributed_load, vx_left, vy_left, brake_force)
        fr_x, fr_y = self.fr_tyre.get_force(distributed_load, vx_right, vy_right, brake_force)
        rl_x, rl_y = self.fr_tyre.get_force(distributed_load, self.vx, self.vy, engine_force/2 + brake_force)
        rr_x, rr_y = self.fr_tyre.get_force(distributed_load, self.vx, self.vy, engine_force/2 + brake_force)

        v_x = 1
        v_y = 1
        v_w = 1
        return v_x, v_y, v_w

    def get_steering(self):
        return self.steering
