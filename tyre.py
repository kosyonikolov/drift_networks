class Tyre:
    """Physics model of a tyre"""

    def __init__(self, radius, max_lat_f, max_lon_f, traction_coefficient):
        self.radius = radius
        self.max_lat_f = max_lat_f
        self.max_lon_f = max_lon_f
        self.traction_coefficient = traction_coefficient

    def get_force(self, f_load, v_x, v_y, car_input):
        f_x = 1 * self.max_lon_f
        f_y = 1 * self.max_lat_f
        return f_x, f_y

    def get_radius(self):
        return self.radius

