import math
import pyglet
from pyglet import shapes
from pyglet.window import key

from car import Car

joystick = None

window = pyglet.window.Window(1200, 600)
window.set_vsync(True)
batch = pyglet.graphics.Batch()

keys = key.KeyStateHandler()
window.push_handlers(keys)

CAR_WIDTH=45
CAR_LENGTH=75
TYRE_WIDTH=10
TYRE_LENGTH=20

TURN_RAD_MIN=100.0
CURV_MAX=1.0/TURN_RAD_MIN
CURV_MIN=-CURV_MAX
CURV_STEP=CURV_MAX/50.0

RAD2DEG = 180.0 / math.pi

ACC_STEP = 3

cx = 600
cy = 300

# radians
rot = 0.0

# curvature, 1 / r (pix)
curv = 0.0

speed = 0

carRect = shapes.Rectangle(cx, cy, CAR_WIDTH, CAR_LENGTH, color=(255, 128, 0), batch=batch)
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

def moveCar():
    global speed
    global rot
    global curv
    global cx
    global cy

    dt = 1.0 / 60.0

    if abs(curv) < 1e-6:
        curv = 0.0

        vx = speed * math.cos(rot)
        vy = speed * math.sin(rot)

        cx += vy * dt
        cy += vx * dt
    else:
        r = 1.0 / curv
        dist = speed * dt
        angle = -dist / r

        # center of turn
        ox, oy = rotate(r, -CAR_LENGTH / 2, rot)
        ox += cx
        oy += cy

        tc.x = ox
        tc.y = oy

        cx -= ox
        cy -= oy

        cx, cy = rotate(cx, cy, angle)

        cx += ox
        cy += oy

        rot += angle

    speed *= 0.985

def update(a):
    global rot
    global curv
    global speed
    global joystick

    gas = 0
    brake = 0
    steer = 0

    if joystick != None:
        gas = 1.0 - 0.5 * (joystick.z + 1.0)
        brake = 1.0 - 0.5 * (joystick.rz + 1.0)
        steer = joystick.x
        print(steer, gas, brake)

    
    speed += ACC_STEP * (gas - brake)
    curv = CURV_MAX * steer

    # if keys[key.UP]:
    #     speed += ACC_STEP
    # if keys[key.DOWN]:
    #     speed -= ACC_STEP

    # if keys[key.LEFT]:
    #     curv = max(curv - CURV_STEP, CURV_MIN)
    # if keys[key.RIGHT]:
    #     curv = min(curv + CURV_STEP, CURV_MAX)

    moveCar()

    carRect.x = cx
    carRect.y = cy
    carRect.rotation = -rot * RAD2DEG

    tx, ty = rotate(CAR_WIDTH / 2.0, CAR_LENGTH / 2.0, -rot)
    tx1, ty1 = rotate(CAR_WIDTH / 2.0, -CAR_LENGTH / 2.0, -rot)

    alpha = 0
    beta = 0

    if curv != 0.0:
        r = 1.0 / curv
        alpha = -math.atan(CAR_LENGTH / (r + CAR_WIDTH / 2))
        beta = -math.atan(CAR_LENGTH / (r - CAR_WIDTH / 2))

    tyres[0].x = cx - tx
    tyres[0].y = cy + ty
    tyres[0].rotation = -(rot + alpha) * RAD2DEG

    tyres[1].x = cx + tx1
    tyres[1].y = cy - ty1
    tyres[1].rotation = -(rot + beta) * RAD2DEG

    tyres[2].x = cx - tx1
    tyres[2].y = cy + ty1
    tyres[2].rotation = -rot * RAD2DEG

    tyres[3].x = cx + tx
    tyres[3].y = cy - ty
    tyres[3].rotation = -rot * RAD2DEG

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

