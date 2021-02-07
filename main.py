import math
import numpy as np
import sys
import pyglet
from pyglet import shapes
from pyglet.window import key

from car import CarV2
from segmentTracker import SegmentTracker

# =============== Graphics and control stuff ===============
WINDOW_WIDTH = 1700
WINDOW_HEIGHT = 800

RAD2DEG = 180.0 / math.pi
PIXELS_PER_METER = 8

window = pyglet.window.Window(WINDOW_WIDTH, WINDOW_HEIGHT)
window.set_vsync(True)
batch = pyglet.graphics.Batch()
joystick = None

# in world coord system, meters
cam_pos_x = 0
cam_pos_y = 0

keys = key.KeyStateHandler()
window.push_handlers(keys)

car_v2 = CarV2()

# open track and draw
track = np.loadtxt(sys.argv[1])
n_track = len(track)

tracker = SegmentTracker(track)

CAR_LENGTH = car_v2.length * PIXELS_PER_METER
CAR_WIDTH = car_v2.width * PIXELS_PER_METER
TYRE_WIDTH = car_v2.tire_width * PIXELS_PER_METER
TYRE_LENGTH = car_v2.tire_length * PIXELS_PER_METER

N_TRAIL = 200

carRect = shapes.Rectangle(0, 0, CAR_WIDTH, CAR_LENGTH, color=(255, 128, 0), batch=batch)
carRect.anchor_x = CAR_WIDTH / 2
carRect.anchor_y = CAR_LENGTH / 2

pt_on_seg_circle = shapes.Circle(0, 0, 5, color = (0, 255, 0), batch=batch)
seg_line = shapes.Line(0, 0, 1, 1, 2, color = (0, 0, 255), batch=batch)

trail_points = [shapes.Circle(0, 0, 3, 7, (
int(240 * x / (N_TRAIL - 1)), int(240 * x / (N_TRAIL - 1)), int(240 * x / (N_TRAIL - 1))), batch=batch) for x in
                range(N_TRAIL)]

colors = [(64, 64, 64), (64, 64, 64), (64, 64, 64), (64, 64, 64)]

tyres = [shapes.Rectangle(110, 110, TYRE_WIDTH, TYRE_LENGTH, color=colors[i], batch=batch) for i in range(4)]
for i in range(4):
    tyres[i].anchor_x = TYRE_WIDTH / 2
    tyres[i].anchor_y = TYRE_LENGTH / 2


def rotate(x, y, angle):
    sinA = math.sin(angle)
    cosA = math.cos(angle)

    return x * cosA - y * sinA, x * sinA + y * cosA

# for keyboard input
gas_old = 0
brake_old = 0
steer_old = 0

# constant for IIR input filter
keyboard_k = 0.8


def update(a):
    global joystick
    global gas_old
    global brake_old
    global steer_old
    global N_TRAIL

    if joystick is not None:
        gas = 1.0 - 0.5 * (joystick.z + 1.0)
        brake = 1.0 - 0.5 * (joystick.rz + 1.0)
        steer = max(-1, min(1, 900 * joystick.x / 360))
    else:
        # keyboard input
        gas_in = 1 if keys[key.UP] else 0
        brake_in = 1 if keys[key.SPACE] else 0
        steer_in = keys[key.RIGHT] - keys[key.LEFT]

        # apply iir filter
        gas_old = gas_old * keyboard_k + (1.0 - keyboard_k) * gas_in
        brake_old = brake_old * keyboard_k + (1.0 - keyboard_k) * brake_in
        steer_old = steer_old * keyboard_k + (1.0 - keyboard_k) * steer_in

        gas = gas_old
        brake = brake_old
        steer = steer_old

    if abs(steer) < 0.00001:
        steer = 0
    if abs(brake) < 0.00001:
        brake = 0
    if abs(gas) < 0.00001:
        gas = 0

    steering_angle = steer * 3.14 / 4.0
    car_v2.update(steering_angle, gas * 100, brake * 100, False, False)
    #print("gas: {}  brake: {}  steering: {}".format(gas * 100, brake * 100, steering_angle))

    cx = carRect.x = car_v2.position.x * PIXELS_PER_METER
    cy = carRect.y = car_v2.position.y * PIXELS_PER_METER
    car_angle = car_v2.angle * -1

    # update trail
    for i in range(N_TRAIL - 1):
        trail_points[i].x = trail_points[i + 1].x
        trail_points[i].y = trail_points[i + 1].y

    trail_points[N_TRAIL - 1].x = cx
    trail_points[N_TRAIL - 1].y = cy

    # update tracker
    dist_to_seg, pt_on_seg, seg_point_0, seg_point_1 = tracker.update(car_v2.position.x, car_v2.position.y)
    seg_line.x  = seg_point_0[0] * PIXELS_PER_METER
    seg_line.y  = seg_point_0[1] * PIXELS_PER_METER
    seg_line.x2 = seg_point_1[0] * PIXELS_PER_METER
    seg_line.y2 = seg_point_1[1] * PIXELS_PER_METER
    pt_on_seg_circle.x = pt_on_seg[0] * PIXELS_PER_METER
    pt_on_seg_circle.y = pt_on_seg[1] * PIXELS_PER_METER

    carRect.rotation = -1 * car_angle * RAD2DEG

    tx, ty = rotate(CAR_WIDTH / 2.0, CAR_LENGTH / 2.0, -car_angle)
    tx1, ty1 = rotate(CAR_WIDTH / 2.0, -CAR_LENGTH / 2.0, -car_angle)

    alpha = -steering_angle
    beta = -steering_angle

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

    # rot += 0.01


@window.event
def on_draw():
    window.clear()
    batch.draw()

    cx = carRect.x = car_v2.position.x * PIXELS_PER_METER
    cy = carRect.y = car_v2.position.y * PIXELS_PER_METER

    pyglet.gl.glLoadIdentity()
    pyglet.gl.glTranslatef(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0)
    pyglet.gl.glTranslatef(-cx, -cy, 0)


joysticks = pyglet.input.get_joysticks()
if joysticks:
    joystick = joysticks[0]
    joystick.open()

track_points = [
    shapes.Circle(track[i][0] * PIXELS_PER_METER, track[i][1] * PIXELS_PER_METER, 3, 4, (255, 0, 0), batch=batch) for i
    in range(len(track))]

track_line = []
track_line.append(track_points[0].x)
track_line.append(track_points[0].y)
for i in range(n_track):
    track_line.append(track_points[i].x)
    track_line.append(track_points[i].y)
track_line.append(track_points[n_track - 1].x)
track_line.append(track_points[n_track - 1].y)

track_line_grp = pyglet.graphics.Group()
batch.add(len(track_line) // 2, pyglet.gl.GL_LINE_STRIP, track_line_grp, ("v2f", track_line))

pyglet.clock.schedule_interval(update, 1.0 / 120)
pyglet.app.run()
