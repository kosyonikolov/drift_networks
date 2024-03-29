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
N_NEXT = 8
PT_INTERVAL = 10

carRect = shapes.Rectangle(0, 0, CAR_WIDTH, CAR_LENGTH, color=(255, 128, 0), batch=batch)
carRect.anchor_x = CAR_WIDTH / 2
carRect.anchor_y = CAR_LENGTH / 2

pt_on_seg_circle = shapes.Circle(0, 0, 5, color = (0, 255, 0), batch=batch)
seg_line = shapes.Line(0, 0, 1, 1, 2, color = (0, 0, 255), batch=batch)

trail_points = [shapes.Circle(0, 0, 3, 7, (
int(240 * x / (N_TRAIL - 1)), int(240 * x / (N_TRAIL - 1)), int(240 * x / (N_TRAIL - 1))), batch=batch) for x in
                range(N_TRAIL)]

next_points_circles = [shapes.Circle(0, 0, 5, color=(255, 128, 0), batch=batch) for x in range(N_NEXT)]
next_points_lines = [shapes.Line(0, 0, 1, 1, 1, color=(255,128,0), batch=batch) for x in range(N_NEXT)]

velocity_line = shapes.Line(0, 0, 0, 0, 3, color=(0, 128, 255), batch=batch)
body_line = shapes.Line(0, 0, 0, 0, 3, color=(0, 255, 128), batch=None)
track_v_line = shapes.Line(0, 0, 0, 0, 3, color=(255, 0, 128), batch=None)

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
    car_v2.update(steering_angle, gas * 100, brake * 100, False, False)
    car_v2.update(steering_angle, gas * 100, brake * 100, False, False)
    car_v2.update(steering_angle, gas * 100, brake * 100, False, False)
    #print("gas: {}  brake: {}  steering: {}".format(gas * 100, brake * 100, steering_angle))

    #print("{0:.3f}, {1:.3f}".format(car_v2.velocity.x, car_v2.velocity.y))
    #v_angle = math.atan2(car_v2.velocity.x, car_v2.velocity.y)
    
    velocity_vec  = np.array([car_v2.velocity.x, car_v2.velocity.y])
    body_norm_vec = np.array([math.sin(car_v2.angle), math.cos(car_v2.angle)])
    body_velocity = np.dot(velocity_vec, body_norm_vec)
    velocity      = np.linalg.norm(velocity_vec)

    #print("{0:.3f}, {1:.3f}".format(velocity, body_velocity))

    cx = carRect.x = car_v2.position.x * PIXELS_PER_METER
    cy = carRect.y = car_v2.position.y * PIXELS_PER_METER
    car_angle = car_v2.angle * -1

    velocity_line.x = cx
    velocity_line.y = cy
    velocity_line.x2 = (car_v2.position.x + velocity_vec[0]) * PIXELS_PER_METER
    velocity_line.y2 = (car_v2.position.y + velocity_vec[1]) * PIXELS_PER_METER

    body_line.x = cx
    body_line.y = cy
    body_line.x2 = (car_v2.position.x + 5 * body_norm_vec[0]) * PIXELS_PER_METER
    body_line.y2 = (car_v2.position.y + 5 * body_norm_vec[1]) * PIXELS_PER_METER

    # update trail
    for i in range(N_TRAIL - 1):
        trail_points[i].x = trail_points[i + 1].x
        trail_points[i].y = trail_points[i + 1].y

    trail_points[N_TRAIL - 1].x = cx
    trail_points[N_TRAIL - 1].y = cy

    # update tracker
    dist_to_seg, pt_on_seg, seg_point_0, seg_point_1, next_points = tracker.update(car_v2.position.x, car_v2.position.y, N_NEXT, PT_INTERVAL)
    seg_line.x  = seg_point_0[0] * PIXELS_PER_METER
    seg_line.y  = seg_point_0[1] * PIXELS_PER_METER
    seg_line.x2 = seg_point_1[0] * PIXELS_PER_METER
    seg_line.y2 = seg_point_1[1] * PIXELS_PER_METER
    pt_on_seg_circle.x = pt_on_seg[0] * PIXELS_PER_METER
    pt_on_seg_circle.y = pt_on_seg[1] * PIXELS_PER_METER

    track_vec = next_points[0] - pt_on_seg
    track_vec = track_vec / np.linalg.norm(track_vec)
    track_v = np.dot(velocity_vec, track_vec)
    print("{0:.2f}".format(track_v))

    track_v_line.x = cx
    track_v_line.y = cy
    track_v_line.x2 = (car_v2.position.x + track_v * track_vec[0]) * PIXELS_PER_METER
    track_v_line.y2 = (car_v2.position.y + track_v * track_vec[1]) * PIXELS_PER_METER

    for i in range(N_NEXT):
        next_points_lines[i].x = cx
        next_points_lines[i].y = cy

        next_points_lines[i].x2 = next_points_circles[i].x = next_points[i][0] * PIXELS_PER_METER
        next_points_lines[i].y2 = next_points_circles[i].y = next_points[i][1] * PIXELS_PER_METER        

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
    pyglet.gl.glLoadIdentity()
    pyglet.gl.glTranslatef(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0)
    pyglet.gl.glTranslatef(-cx, -cy, 0)

@window.event
def on_draw():
    window.clear()
    batch.draw()


joysticks = pyglet.input.get_joysticks()
if joysticks:
    joystick = joysticks[0]
    joystick.open()

track_start_vec   = track[1] - track[0]
track_start_angle = math.atan2(track_start_vec[0], track_start_vec[1])
car_v2.angle = track_start_angle
car_v2.position.x = track[0][0]
car_v2.position.y = track[0][1]

track_start_vec = track_start_vec / np.linalg.norm(track_start_vec)
car_v2.velocity.x = 15 * track_start_vec[0]
car_v2.velocity.y = 15 * track_start_vec[1]

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

pyglet.clock.schedule_interval(update, 1.0 / 30)
pyglet.app.run()
