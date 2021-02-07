import math
import numpy as np
import sys
import pyglet
from pyglet import shapes
from pyglet.window import key

# load track
track = np.loadtxt(sys.argv[1])
n_track = len(track)

# load episode
episode = np.loadtxt(sys.argv[2])
n_frames = len(episode)
frame_idx = 0

# =============== Graphics ===============
WINDOW_WIDTH = 1700
WINDOW_HEIGHT = 800

RAD2DEG = 180.0 / math.pi
PIXELS_PER_METER = 8

CAR_LENGTH = 4.0 * PIXELS_PER_METER
CAR_WIDTH = 1.8 * PIXELS_PER_METER
TYRE_WIDTH = 0.5 * PIXELS_PER_METER
TYRE_LENGTH = 1 * PIXELS_PER_METER

N_TRAIL = 200

window = pyglet.window.Window(WINDOW_WIDTH, WINDOW_HEIGHT)
window.set_vsync(True)
batch = pyglet.graphics.Batch()

carRect = shapes.Rectangle(0, 0, CAR_WIDTH, CAR_LENGTH, color=(255, 128, 0), batch=batch)
carRect.anchor_x = CAR_WIDTH / 2
carRect.anchor_y = CAR_LENGTH / 2

trail_points = [shapes.Circle(0, 0, 3, 7, (
int(240 * x / (N_TRAIL - 1)), int(240 * x / (N_TRAIL - 1)), int(240 * x / (N_TRAIL - 1))), batch=batch) for x in
                range(N_TRAIL)]

colors = [(64, 64, 64), (64, 64, 64), (64, 64, 64), (64, 64, 64)]

tyres = [shapes.Rectangle(110, 110, TYRE_WIDTH, TYRE_LENGTH, color=colors[i], batch=batch) for i in range(4)]
for i in range(4):
    tyres[i].anchor_x = TYRE_WIDTH / 2
    tyres[i].anchor_y = TYRE_LENGTH / 2

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

def rotate(x, y, angle):
    sinA = math.sin(angle)
    cosA = math.cos(angle)

    return x * cosA - y * sinA, x * sinA + y * cosA

def update(a):
    global frame_idx

    car_x, car_y, car_angle, steering_angle = episode[frame_idx]
    frame_idx = (frame_idx + 1) % n_frames

    cx = carRect.x = car_x * PIXELS_PER_METER
    cy = carRect.y = car_y * PIXELS_PER_METER
    car_angle = -car_angle

    # update trail
    for i in range(N_TRAIL - 1):
        trail_points[i].x = trail_points[i + 1].x
        trail_points[i].y = trail_points[i + 1].y

    trail_points[N_TRAIL - 1].x = cx
    trail_points[N_TRAIL - 1].y = cy

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

    pyglet.gl.glLoadIdentity()
    pyglet.gl.glTranslatef(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2, 0)
    pyglet.gl.glTranslatef(-cx, -cy, 0)


@window.event
def on_draw():
    window.clear()
    batch.draw()

pyglet.clock.schedule_interval(update, 1.0 / 30)
pyglet.app.run()
