import math
import pyglet
from pyglet import shapes
from pyglet.window import key

from car import Car
from engine import Engine
from tyre import Tyre
from world import World


# =============== Graphics and control stuff ===============
WINDOW_WIDTH  = 1700
WINDOW_HEIGHT = 800

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

window = pyglet.window.Window(WINDOW_WIDTH, WINDOW_HEIGHT)
window.set_vsync(True)
batch = pyglet.graphics.Batch()
joystick = None

# in world coord system, meters
cam_pos_x = 0
cam_pos_y = 0

keys = key.KeyStateHandler()
window.push_handlers(keys)

# =============== Simulation stuff ===============
engine_force_lut = [15000, 15000, 15000, 15000, 15000, 0]
engine = Engine(550, engine_force_lut)
tyre = Tyre(0.2, 20000, 10000, 1.0, 5.0, 0.05)
car = Car(engine, 10000, 2, 4, 1800, tyre)
world = World(car)

N_TRAIL = 300

carRect = shapes.Rectangle(0, 0, CAR_WIDTH, CAR_LENGTH, color=(255, 128, 0), batch=batch)
carRect.anchor_x = CAR_WIDTH / 2
carRect.anchor_y = CAR_LENGTH / 2

trail_points = [shapes.Circle(0, 0, 3, 7, (int(255 * x / (N_TRAIL - 1)),int(255 * x / (N_TRAIL - 1)),int(255 * x / (N_TRAIL - 1))), batch=batch) for x in range(N_TRAIL)]

colors = [(64,64,64), (64,64,64), (64,64,64), (64,64,64)]

tyres = [shapes.Rectangle(110, 110, TYRE_WIDTH, TYRE_LENGTH, color=colors[i], batch=batch) for i in range(4)]
for i in range(4):
    tyres[i].anchor_x = TYRE_WIDTH / 2
    tyres[i].anchor_y = TYRE_LENGTH / 2

def rotate(x, y, angle):
    sinA = math.sin(angle)
    cosA = math.cos(angle)

    return x * cosA - y * sinA, x * sinA + y * cosA

# for keyboard input
gas_old   = 0
brake_old = 0
steer_old = 0

# constant for IIR input filter
keyboard_k = 0.6

def update(a):
    global joystick
    global gas_old
    global brake_old
    global steer_old
    global N_TRAIL

    gas = 0
    brake = 0
    steer = 0

    if joystick != None:
        gas = 1.0 - 0.5 * (joystick.z + 1.0)
        brake = 1.0 - 0.5 * (joystick.rz + 1.0)
        steer = joystick.x
    else:
        # keyboard input
        gas_in   = 1 if keys[key.UP] else 0
        brake_in = 1 if keys[key.DOWN] else 0
        steer_in = keys[key.RIGHT] - keys[key.LEFT]

        # apply iir filter
        gas_old = gas_old * keyboard_k + (1.0 - keyboard_k) * gas_in
        brake_old = brake_old * keyboard_k + (1.0 - keyboard_k) * brake_in
        steer_old = steer_old * keyboard_k + (1.0 - keyboard_k) * steer_in

        gas = gas_old
        brake = brake_old
        steer = steer_old

    #print("{0:.3f}\t{1:.3f}\t{2:.3f}".format(steer, gas, brake))

    # TODO give inputs to car
    world.update(1.0/120, gas, brake, -steer)

    car_x, car_y, car_angle, steering_angle = world.get_car_position()

    cx = carRect.x = car_x * PIXELS_PER_METER
    cy = carRect.y = car_y * PIXELS_PER_METER

    # update trail
    for i in range(N_TRAIL - 1):
        trail_points[i].x = trail_points[i + 1].x
        trail_points[i].y = trail_points[i + 1].y

    trail_points[N_TRAIL - 1].x = cx
    trail_points[N_TRAIL - 1].y = cy


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

    pyglet.gl.glLoadIdentity()
    pyglet.gl.glTranslatef(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0)

def lalal():
    return 1,2,3


joysticks = pyglet.input.get_joysticks()
if joysticks:
    joystick = joysticks[0]
    joystick.open()

pyglet.clock.schedule_interval(update, 1.0/120)
pyglet.app.run()

