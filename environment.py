import numpy as np
from car import CarV2
from segmentTracker import SegmentTracker

class Environment:
    def __init__(self, track, track_points = 8, point_interval = 10, max_dist = 30, initial_velocity = 15, dump_dir = "dump/episodes/"):
        self.car = CarV2()
        self.track = track
        self.segment_tracker = SegmentTracker(track)
        self.phys_calls_per_update = 4

    def update(self, steering_angle, throttle, brake, front_slip, rear_slip):
        self.car.update(steering_angle, throttle, brake, front_slip, rear_slip)

    def reset(self):
        self.car.reset()

    def get_state(self):
        return to4ki po pistata

    def get_reward(self):