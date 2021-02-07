import numpy as np

class SegmentTracker:
    def __init__(self, track):
        self.track = track

        # a segment is defined by its start point index
        # if the point is the last, the next point is assumed 
        # to be the first in the track
        self.segment_id = 0

        # true if the car has entered the line segment
        # that is, ratio >= 0 and ratio < 1
        self.segment_started = False

    # dist_to_seg, pt_on_seg, seg_point_0, seg_point_1
    def update(self, car_x, car_y):
        id_start = self.segment_id
        id_end = (id_start + 1) % len(self.track)

        seg_point_0 = self.track[self.segment_id]
        seg_point_1 = self.track[id_end]
        car_point = np.array([car_x, car_y])

        seg_vector = seg_point_1 - seg_point_0
        seg_length = np.linalg.norm(seg_vector)
        seg_norm_vector = seg_vector / seg_length

        car_vector = car_point - seg_point_0

        # length of projection of car_vector onto the segment vector
        proj_length = np.dot(car_vector, seg_norm_vector)

        # 0..1 -> inside segment
        # < 0  -> hasn't started
        # > 1  -> finished
        seg_ratio = proj_length / seg_length

        if seg_ratio >= 0 and seg_ratio <= 1:
            self.segment_started = True

        if seg_ratio >= 1.0 and self.segment_started:
            # load next segment
            self.segment_id = id_end
            # recalculate for next segment
            self.segment_started = False
            return self.update(car_x, car_y)

        seg_ratio = min(1, max(0, seg_ratio))
        
        pt_on_seg   = seg_point_0 + seg_ratio * seg_vector
        dist_to_seg = np.linalg.norm(pt_on_seg - car_point)

        return dist_to_seg, pt_on_seg, seg_point_0, seg_point_1
