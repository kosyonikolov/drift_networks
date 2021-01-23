import math

class Engine:
    def __init__(self, max_rpm, force_lut):
        self.max_rpm = max_rpm
        self.force_lut = force_lut

    def get_force(self, rpm, throttle):
        max_force = self.get_max_force(rpm)
        return throttle * max_force

    def get_max_force(self, rpm):
        lut_index_max = len(self.force_lut) - 1
        lut_index_fract = lut_index_max * rpm / self.max_rpm
        
        if lut_index_fract > lut_index_max:
            return 0 # out of rev range
        
        index_low  = math.floor(lut_index_fract)
        index_high = math.ceil(lut_index_fract)
        k = lut_index_fract - index_low
        return self.force_lut[index_low] * (1.0 - k) + k * self.force_lut[index_high]