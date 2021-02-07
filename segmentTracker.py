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
        self.segment_started = True

    def reset(self):
        self.segment_id = 0
        self.segment_started = True # set to false to prevent corner cutting

    # dist_to_seg, pt_on_seg, seg_point_0, seg_point_1, next_points
    def update(self, car_x, car_y, n_next_points, point_interval):
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
            self.segment_started = True
            return self.update(car_x, car_y, n_next_points, point_interval)

        seg_ratio = min(1, max(0, seg_ratio))
        
        pt_on_seg   = seg_point_0 + seg_ratio * seg_vector
        dist_to_seg = np.linalg.norm(pt_on_seg - car_point)

        # recalculate projection length with clipped ratio
        proj_length = seg_ratio * seg_length

        # calculate next points, spaced evenly on the track
        next_points = []
        prev_point = pt_on_seg
        prev_seg = id_start
        for i in range(n_next_points):
            dist_left = point_interval
            
            # first, check if we can fit in the current segment
            seg_istart = prev_seg
            seg_iend   = (prev_seg + 1) % len(self.track)

            seg_start = self.track[seg_istart]
            seg_end   = self.track[seg_iend]

            seg_vector = seg_end - seg_start
            seg_vector = seg_vector / np.linalg.norm(seg_vector)
            
            dist_to_end = np.linalg.norm(seg_end - prev_point)
            if dist_to_end >= dist_left:
                # we can fit in the current segment
                curr_point = prev_point + dist_left * seg_vector
                next_points.append(curr_point)
                prev_point = curr_point
                # prev_seg stays the same
                continue
        
            dist_left -= dist_to_end
            seg_istart = (seg_istart + 1) % len(self.track)
            # move segments until one can fit the next point
            while True:
                seg_iend = (seg_istart + 1) % len(self.track)
                seg_start = self.track[seg_istart]
                seg_end   = self.track[seg_iend]
                
                seg_vector = seg_end - seg_start
                seg_len    = np.linalg.norm(seg_vector)
                seg_vector = seg_vector / seg_len

                if (dist_left <= seg_len):
                    # we can fit
                    curr_point = seg_start + dist_left * seg_vector
                    next_points.append(curr_point)
                    prev_point = curr_point
                    prev_seg   = seg_istart
                    break

                # more distance left, move to next segment
                dist_left -= seg_len
                seg_istart = seg_iend

        return dist_to_seg, pt_on_seg, seg_point_0, seg_point_1, next_points
