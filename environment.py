import math
import numpy as np
from car import CarV2
from segmentTracker import SegmentTracker

class Environment:
    def __init__(self, track, track_points = 8, point_interval = 10, max_dist = 30, zero_reward_dist = 15, speed_power_reward = 2.0, initial_velocity = 15, dump_dir = "dump/episodes/"):
        self.car = CarV2()
        self.track = track
        self.track_points = track_points
        self.point_interval = point_interval
        self.max_dist = max_dist

        # constant for reward paraboloid
        self.a_reward = -1 / zero_reward_dist**2
        self.speed_power_reward = speed_power_reward
        self.initial_velocity = initial_velocity
        self.dump_dir = dump_dir

        self.segment_tracker = SegmentTracker(track)
        self.phys_calls_per_update = 4
        self.done = False

        self.reset()

    def update(self, steering_angle, throttle, brake, front_slip, rear_slip):
        for i in range(self.phys_calls_per_update):
            self.car.update(steering_angle, throttle, brake, front_slip, rear_slip)

    def reset(self):
        self.car.reset()
        self.segment_tracker.reset()
        self.done = False

    def get_state(self):
        car_pos = self.car.position
        dist_to_seg, pt_on_seg, seg_point_0, seg_point_1, next_points = self.segment_tracker.update(car_pos.x, car_pos.y, self.track_points, self.point_interval)
        self.done |= self.segment_tracker.passed_end
        self.done |= dist_to_seg > self.max_dist

        velocity_vec  = np.array([self.car.velocity.x, self.car.velocity.y])
        body_norm_vec = np.array([math.sin(self.car.angle), math.cos(self.car.angle)])
                
        # vector to projection on track
        car_to_track = pt_on_seg - np.array([self.car.position.x, self.car.position.y])

        # transform everything into car coordinate system
        car_up    = body_norm_vec
        car_right = np.array([car_up[1], -car_up[0]])

        def to_car(vec):
            return np.array([np.dot(car_right, vec), np.dot(car_up, vec)])

        car_velocity_vec = to_car(velocity_vec)
        car_to_track     = to_car(car_to_track)
        car_track_points = [to_car(point) for point in next_points]
        
        def to_polar(vec):
            rho = np.linalg.norm(vec)
            phi = math.atan2(vec[1], vec[0])
            return np.array([rho, phi])

        # fill final state array - in polar coord system
        state_list = [to_polar(car_velocity_vec), to_polar(car_to_track)]
        for track_pt in car_track_points:
            state_list.append(to_polar(track_pt))

        return state_list, self.done

    def get_reward(self):
        car_pos = self.car.position
        dist_to_seg, pt_on_seg, seg_point_0, seg_point_1, next_points = self.segment_tracker.update(car_pos.x, car_pos.y, self.track_points, self.point_interval)

        velocity_vec  = np.array([self.car.velocity.x, self.car.velocity.y])
        track_vec     = next_points[0] - pt_on_seg
        track_vec     = track_vec / np.linalg.norm(track_vec)

        track_velocity = np.dot(velocity_vec, track_vec)

        reward = track_velocity ** self.speed_power_reward
        # reduce reward based on distance
        # after some distance, it becomes negative
        ratio = self.calc_reward_ratio(dist_to_seg)
        reward *= ratio
        return reward

    def calc_reward_ratio(self, dist):
        return self.a_reward * (dist**2) + 1