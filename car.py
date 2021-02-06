import math
from util import Vec2

# PHYSICS ASSUMPTIONS (Arbitraty values)
AIR_DRAG = 5.0
ROLLING_RESISTENCE = 30.0
CA_R = -15.2  # cornering stiffness rear
CA_F = -13.5  # cornering stiffness front
MAX_GRIP = 10.0
dT = 1/120   # refresh rate

FORCE_MULTIPLIER = 600
BRAKE_COEF = 0.1


def sign(val):
    if val < 0.0:
        return -1

    return val


class CarV2:
    def __init__(self):

        # static data in SI
        self.front_axle_to_cg = 2.0
        self.rear_axle_to_cg = 2.0
        self.wheelbase = self.rear_axle_to_cg + self.front_axle_to_cg
        self.mass = 1500
        self.inertia = 1500
        self.width = 1.8
        self.length = 4.0
        self.tire_width = 0.5
        self.tire_length = 1

        # dynamic data in SI

        # position in World coordinates
        self.position = Vec2(0, 0)

        # velocity in World coordinates
        self.velocity = Vec2(0, 0)

        # angle in World coordinates
        self.angle = 0

        self.angular_v = 0

    def update(self, steerangle, throttle, brake, front_slip, rear_slip):
        sn = math.sin(self.angle)
        cs = math.cos(self.angle)

        # transform velocity to car reference frame
        velocity = Vec2(
            cs * self.velocity.y + sn * self.velocity.x,
            -sn * self.velocity.y + cs * self.velocity.x
        )

        # Lateral force on wheels
        # Resulting velocity of the wheels as result of the yaw rate of the car body
        # v = yawrate * r where r is distance of wheel to CG (approx. half wheel base)
        #
        yaw_speed = (self.wheelbase / 2) * self.angular_v

        rot_angle = math.atan2(yaw_speed, velocity.x)

        # calculate side slip angle of the car
        sideslip = 0
        if velocity.x != 0:
            sideslip = math.atan(velocity.y / velocity.x)

        # calculate side slip of front and rear wheels
        slip_angle_front = sideslip + rot_angle - steerangle
        slip_angle_rear  = sideslip - rot_angle

        # weight per axle = mass/2 times 1G
        weight = self.mass * 9.8 * 0.5

        # lateral force on front wheels = (Ca * slip angle) capped to friction circle * load
        flatf = Vec2(0, CA_F * slip_angle_front)
        flatf.y = min(MAX_GRIP, flatf.y)
        flatf.y = max(-MAX_GRIP, flatf.y)
        flatf.y *= weight

        if front_slip:
            flatf.y *= 0.5

        # lateral force on rear wheels
        flatr = Vec2(0, CA_R * slip_angle_rear)
        flatr.y = min(MAX_GRIP, flatr.y)
        flatr.y = max(-MAX_GRIP, flatr.y)
        flatr.y *= weight

        if rear_slip:
            flatr.y *= 0.5

        # longitudinal force on rear wheels
        ftraction = Vec2(
            FORCE_MULTIPLIER * (throttle - BRAKE_COEF * brake * sign(velocity.x)),
            0
        )
        if rear_slip:
            ftraction.x *= 0.5

        # Forces and torque on body

        # drag and rolling resistance
        resistance = Vec2(
            -(ROLLING_RESISTENCE * velocity.x + AIR_DRAG * velocity.x * math.fabs(velocity.x)),
            -(ROLLING_RESISTENCE * velocity.y + AIR_DRAG * velocity.y * math.fabs(velocity.y))
        )

        # sum forces
        force = Vec2(
            (ftraction.x + math.sin(steerangle) * flatf.x + flatr.x + resistance.x),
            (ftraction.y + math.cos(steerangle) * flatf.y + flatr.y + resistance.y)
        )

        # torque on body from lat forces
        torque = self.front_axle_to_cg * flatf.y - self.rear_axle_to_cg * flatr.y

        # Acceleration

        # Newton
        acceleration = Vec2(
            force.x / self.mass,
            force.y / self.mass
        )
        angular_acceleration = torque / self.inertia

        # velocity and position

        # transform to world vectors
        acceleration_world = Vec2(
            cs * acceleration.y + sn * acceleration.x,
            -sn * acceleration.y + cs * acceleration.x
        )

        # velocity = integrated acceleration
        self.velocity.x += dT * acceleration_world.x
        self.velocity.y += dT * acceleration_world.y

        # position = integrated velocity
        self.position.x += dT * self.velocity.x
        self.position.y += dT * self.velocity.y

        # angular velocity and heading

        # angular velocity += integrated angular acceleration
        self.angular_v += dT * angular_acceleration

        # angle += integrated angular velocity
        self.angle += dT * self.angular_v
