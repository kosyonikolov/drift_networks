import math
from util import rotate_2d


class World:
    def __init__(self, car):
        self.car_x = 0
        self.car_y = 0
        self.car = car
        self.car_angle = 0

    def update(self, dt, gas_input, brake_input, steering_input):
        vx_car, vy_car, vw = self.car.get_velocity(dt, gas_input, brake_input, -steering_input)

        # transform to world coordinates
        vx, vy = rotate_2d(vx_car, vy_car, self.car_angle)

        # darboux sum
        self.car_x += vx * dt
        self.car_y += vy * dt

        slip_angle = math.atan2(vx_car, vy_car)
        
        self.car_angle += vw * dt

        print("slip: {0:.2f}".format((slip_angle) * 180 / 3.14))

        return

    def get_car_position(self):
        return self.car_x, self.car_y, self.car_angle, self.car.get_steering()
