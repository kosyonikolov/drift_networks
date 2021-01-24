import math

class Tyre:
    """Physics model of a tyre"""

    def __init__(self, radius, max_lat_f, max_lon_f, traction_coefficient, cornering_stiffness, crr):
        self.radius = radius
        self.max_lat_f = max_lat_f
        self.max_lon_f = max_lon_f
        self.traction_coefficient = traction_coefficient
        self.cornering_stiffness = cornering_stiffness
        self.crr = crr # coefficient of rolling resistance

    def get_force(self, f_load, vx, vy, car_input):

        # ==== Longitudinal force ====
        f_resist = f_load * vy * self.crr
        fy_raw = car_input - f_resist
        fy_sign = -1 if fy_raw < 0 else 1
        fy = fy_sign * min(math.fabs(fy_raw), self.max_lon_f)

        # ==== Lateral force ====
        slip_angle = math.atan2(vx, vy)
        fx_raw = self.cornering_stiffness * f_load * slip_angle
        fx_sign = -1 if fx_raw < 0 else 1

        # limit sideways friction
        fx = -fx_sign * min(math.fabs(fx_raw), self.max_lat_f)  # the resulting force should oppose the lateral velocity
        
        return fx, fy

    def get_radius(self):
        return self.radius

