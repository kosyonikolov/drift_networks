import math
import pyglet
from pyglet import shapes
from pyglet.window import key

from car import Car
from engine import Engine
from tyre import Tyre
from world import World


# =============== Graphics and control stuff ===============
WINDOW_WIDTH  = 1800
WINDOW_HEIGHT = 1000

RAD2DEG = 180.0 / math.pi
PIXELS_PER_METER = 10

# TODO this should be refactored...
CAR_WIDTH_METERS = 2
CAR_LENGTH_METERS = 4
TYRE_WIDTH_METERS = 0.2
TYRE_LENGTH_METERS = 0.5

CAR_WIDTH   = CAR_WIDTH_METERS   * PIXELS_PER_METER
CAR_LENGTH  = CAR_LENGTH_METERS  * PIXELS_PER_METER
TYRE_WIDTH  = TYRE_WIDTH_METERS  * PIXELS_PER_METER
TYRE_LENGTH = TYRE_LENGTH_METERS * PIXELS_PER_METER

window = pyglet.window.Window(1800, 1000)
window.set_vsync(True)
batch = pyglet.graphics.Batch()
joystick = None

# in world coord system, meters
cam_pos_x = 0
cam_pos_y = 0

keys = key.KeyStateHandler()
window.push_handlers(keys)

# =============== Simulation stuff ===============
engine_force_lut = [1000, 10000, 10000, 10000, 10000, 0]
engine = Engine(7500, engine_force_lut)
tyre = Tyre(0.2, 10000, 10000, 1.0, 1.0, 0.05)
car = Car(engine, 20000, 2, 4, 1200, tyre)
world = World(car)


carRect = shapes.Rectangle(0, 0, CAR_WIDTH, CAR_LENGTH, color=(255, 128, 0), batch=batch)
carRect.anchor_x = CAR_WIDTH / 2
carRect.anchor_y = CAR_LENGTH / 2

tc = shapes.Circle(400, 400, 5, None, (0, 255, 0), batch=batch)

colors = [(64,64,64), (64,64,64), (64,64,64), (64,64,64)]

tyres = [shapes.Rectangle(110, 110, TYRE_WIDTH, TYRE_LENGTH, color=colors[i], batch=batch) for i in range(4)]
for i in range(4):
    tyres[i].anchor_x = TYRE_WIDTH / 2
    tyres[i].anchor_y = TYRE_LENGTH / 2

def rotate(x, y, angle):
    sinA = math.sin(angle)
    cosA = math.cos(angle)

    return x * cosA - y * sinA, x * sinA + y * cosA

def update(a):
    global joystick

    gas = 0
    brake = 0
    steer = 0

    if joystick != None:
        gas = 1.0 - 0.5 * (joystick.z + 1.0)
        brake = 1.0 - 0.5 * (joystick.rz + 1.0)
        steer = joystick.x
        print(steer, gas, brake)

    # TODO give inputs to car



    # if keys[key.UP]:
    #     speed += ACC_STEP
    # if keys[key.DOWN]:
    #     speed -= ACC_STEP

    # if keys[key.LEFT]:
    #     curv = max(curv - CURV_STEP, CURV_MIN)
    # if keys[key.RIGHT]:
    #     curv = min(curv + CURV_STEP, CURV_MAX)


    car_x, car_y, car_angle, steering_angle = world.get_car_position()

    cx = carRect.x = car_x * PIXELS_PER_METER
    cy = carRect.y = car_y * PIXELS_PER_METER
    carRect.rotation = -car_angle * RAD2DEG

    tx, ty = rotate(CAR_WIDTH / 2.0, CAR_LENGTH / 2.0, -car_angle)
    tx1, ty1 = rotate(CAR_WIDTH / 2.0, -CAR_LENGTH / 2.0, -car_angle)

    alpha = steering_angle
    beta = steering_angle

    tyres[0].x = cx - tx
    tyres[0].y = cy + ty
    tyres[0].rotation = -(car_angle + alpha) * RAD2DEG

    tyres[1].x = cx + tx1
    tyres[1].y = cy - ty1
    tyres[1].rotation = -(car_angle + beta) * RAD2DEG

    tyres[2].x = cx - tx1
    tyres[2].y = cy + ty1
    tyres[2].rotation = -car_angle * RAD2DEG

    tyres[3].x = cx + tx
    tyres[3].y = cy - ty
    tyres[3].rotation = -car_angle * RAD2DEG

    #rot += 0.01


@window.event
def on_draw():
    window.clear()
    batch.draw()

def lalal():
    return 1,2,3


joysticks = pyglet.input.get_joysticks()
if joysticks:
    joystick = joysticks[0]
    joystick.open()

a, b, c = lalal()
print(a)
pyglet.clock.schedule_interval(update, 1.0/120)
pyglet.app.run()

