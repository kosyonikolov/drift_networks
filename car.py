import copy
import math
 
import tyre
import util
 
"""Physics model of a car"""
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
        self.vw = 0
 
        self.half_diagonal = math.sqrt(width*width + height*height) / 2
 
        # we assume shape of the car to be a rectangular plate
        self.moment_of_inertia = (1.0 / 12.0) * mass * (width*width + height*height)
 
        self.sin_angle = self.calc_sin_angle()
        self.cos_angle = self.calc_cos_angle()
 
        self.fl_tyre = copy.deepcopy(tyre_type)
        self.fr_tyre = copy.deepcopy(tyre_type)
        self.rl_tyre = copy.deepcopy(tyre_type)
        self.rr_tyre = copy.deepcopy(tyre_type)
 
    # get_velocity is blah blah blah
    def get_velocity(self, dt, throttle_input, brake_input, steering_input):
        self.steering = steering_input
 
        # assumption for a perfect distribution
        distributed_load = self.mass * Car.g * 0.25
 
        # convert to car coordinate system
        vx_left, vy_left = util.rotate_2d(self.vx, self.vy, self.steering)
        vx_right, vy_right = util.rotate_2d(self.vx, self.vy, self.steering)
 
        # calculate forces applied on tyres by car
        rpm = self.vy / self.fr_tyre.get_radius()
        engine_force = self.engine.get_force(rpm, throttle_input)
        brake_force = - brake_input * self.max_brake_force
 
        # get wheel forces applied on car
        fl_x, fl_y = self.fr_tyre.get_force(distributed_load, vx_left, vy_left, brake_force)
        fr_x, fr_y = self.fr_tyre.get_force(distributed_load, vx_right, vy_right, brake_force)
        rl_x, rl_y = self.fr_tyre.get_force(distributed_load, self.vx, self.vy, engine_force/2 + brake_force)
        rr_x, rr_y = self.fr_tyre.get_force(distributed_load, self.vx, self.vy, engine_force/2 + brake_force)
 
        # update car linear velocity
        total_x_force = fl_x + fr_x + rl_x + rr_x
        total_y_force = fl_y + fr_y + rl_y + rr_y
 
        vx_result = self.calculate_result_velocity(total_x_force, dt)
        vy_result = self.calculate_result_velocity(total_y_force, dt)
 
        self.vx += vx_result
        self.vy += vy_result
 
        # update car angular velocity
 
        # calculate tangent forces
        fl_x_tangent_force = self.calculate_tangent_x(fl_x)
        fr_x_tangent_force = self.calculate_tangent_x(fr_x) * -1
 
        rl_x_tangent_force = self.calculate_tangent_x(rl_x) * -1
        rr_x_tangent_force = self.calculate_tangent_x(rr_x)
 
        fl_y_tangent_force = self.calculate_tangent_y(fl_y) * -1
        fr_y_tangent_force = self.calculate_tangent_y(fr_y)
 
        rl_y_tangent_force = self.calculate_tangent_y(rl_y)
        rr_y_tangent_force = self.calculate_tangent_y(rr_y) * -1
 
        total_tangent_force = fl_x_tangent_force\
                              + fr_x_tangent_force\
                              + rl_x_tangent_force\
                              + rr_x_tangent_force\
                              + fl_y_tangent_force\
                              + fr_y_tangent_force\
                              + rl_y_tangent_force \
                              + rr_y_tangent_force
 
        # calculate torque
        torque = total_tangent_force * self.half_diagonal
        
        # calculate angular acceleration
        angular_acc = torque / self.moment_of_inertia
        
        # update angular velocity 
        w_result = angular_acc * dt
        self.vw += w_result
 
        return self.vx, self.vy, self.vw
 
    def get_steering(self):
        return self.steering
 
    def calculate_result_velocity(self, f, dt):
        acc = f/self.mass
        return acc*dt
 
    def calculate_tangent_x(self, f):
        return self.cos_angle * f
 
    def calculate_tangent_y(self, f):
        return self.sin_angle * f
 
    # angle between Y axis and line to corner of the vehicle
    def calc_sin_angle(self):
        h_2 = self.height / 2
        w_2 = self.width / 2
 
        diagonal = math.sqrt(h_2 * h_2 + w_2 * w_2)
        return w_2 / diagonal
 
    # angle between Y axis and line to corner of the vehicle
    def calc_cos_angle(self):
        h_2 = self.height / 2
        w_2 = self.width / 2
 
        diagonal = math.sqrt(h_2 * h_2 + w_2 * w_2)
        return h_2 / diagonal