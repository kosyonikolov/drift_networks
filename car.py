import copy
import math
import numpy as np
 
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
 
        self.max_steering_angle = 25.0 * math.pi / 180.0
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
        self.steering = self.max_steering_angle * steering_input
 
        # assumption for a perfect distribution
        distributed_load = self.mass * Car.g * 0.25
 
        # convert to car coordinate system
        print(self.vx, self.vy)
        vx_left, vy_left = util.rotate_2d(self.vx, self.vy, self.steering)
        vx_right, vy_right = util.rotate_2d(self.vx, self.vy, self.steering)
        print(vx_left, vx_right)
 
        # calculate forces applied on tyres by car
        rpm = self.vy / self.fr_tyre.get_radius()
        print("rpm = {0:.2f}".format(rpm))
        engine_force = self.engine.get_force(rpm, throttle_input)
        brake_force = - brake_input * self.max_brake_force
 
        # get wheel forces applied on car
        fl_x, fl_y = self.fr_tyre.get_force(distributed_load, vx_left, vy_left, brake_force)
        fr_x, fr_y = self.fr_tyre.get_force(distributed_load, vx_right, vy_right, brake_force)

        # get front tyre forces in car coord sys
        fl_x, fl_y = util.rotate_2d(fl_x, fl_y, -self.steering)
        fr_x, fr_y = util.rotate_2d(fr_x, fr_y, -self.steering)

        rl_x, rl_y = self.fr_tyre.get_force(distributed_load, self.vx, self.vy, engine_force/2 + brake_force)
        rr_x, rr_y = self.fr_tyre.get_force(distributed_load, self.vx, self.vy, engine_force/2 + brake_force)

        print("v: ({0:.2f}, {1:.2f}) ({2:.2f}, {3:.2f}) ({4:.2f}, {5:.2f}) ({6:.2f}, {7:.2f})".format(vx_left, vy_left, vx_right, vy_right, self.vx, self.vy, self.vx, self.vy))
        print("f: ({0:.2f}, {1:.2f}) ({2:.2f}, {3:.2f}) ({4:.2f}, {5:.2f}) ({6:.2f}, {7:.2f})".format(fl_x, fl_y, fr_x, fr_y, rl_x, rl_y, rr_x, rr_y))
 
        # update car linear velocity
        total_x_force = fl_x + fr_x + rl_x + rr_x
        total_y_force = fl_y + fr_y + rl_y + rr_y
 
        vx_result = self.calculate_result_velocity(total_x_force, dt)
        vy_result = self.calculate_result_velocity(total_y_force, dt)
 
        self.vx += vx_result
        self.vy += vy_result
 
        self.vy = max(self.vy, 0)

        # update car angular velocity
        
        # Torque arm-vectors:
        # front-left  (-w / 2, h / 2)
        # front-right (w / 2,  h / 2)
        # rear-left   (-w / 2, -h / 2)
        # rear-right  (w / 2,  h / 2)
        #
        # Given an arm vector (ax, ay) and a force (fx, fy)
        # the torque induced by it is given by (fx, fy) . (-ay, ax)
        # that is, we take the scalar with the perpendicular vector of the arm

        w_2 = self.width / 2.0
        h_2 = self.height / 2.0

        arm_fl = [-w_2, h_2]
        arm_fr = [w_2, h_2]
        arm_rl = [-w_2, -h_2]
        arm_rr = [w_2, -h_2]

        # perpendicular arm vectors
        p_arm_fl = [-arm_fl[1], arm_fl[0]]
        p_arm_fr = [-arm_fr[1], arm_fr[0]]
        p_arm_rl = [-arm_rl[1], arm_rl[0]]
        p_arm_rr = [-arm_rr[1], arm_rr[0]]

        torque_fl = np.dot([fl_x, fl_y], p_arm_fl)
        torque_fr = np.dot([fr_x, fr_y], p_arm_fr)
        torque_rl = np.dot([rl_x, rl_y], p_arm_rl)
        torque_rr = np.dot([rr_x, rr_y], p_arm_rr)

        print("t: {0:.2f}\t{1:.2f}\t{2:.2f}\t{3:.2f}".format(torque_fl, torque_fr, torque_rl, torque_fr))

        # calculate torque
        torque = torque_fl + torque_fr + torque_rl + torque_rr

        print ("-------------")
        print ( torque )
        print ("-------------")

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