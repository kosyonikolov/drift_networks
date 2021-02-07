from environment import Environment
import numpy as np
import sys

# load track
track = np.loadtxt(sys.argv[1])

env = Environment(track)

env.reset(True)

done = False

max_frames = 10000
frame = 0

while not done and frame < max_frames:
    state, done = env.get_state()
    angle_to_first = state[3][1]

    prev = state[0][1]
    diff = 0
    for i in range(5):
        curr = state[i+2][1]
        diff += abs(prev - curr)
        prev = curr

    print("DIFF {}".format(diff))


    print(angle_to_first)

    steer_angle = min(3.14 / 4, max(-3.14 / 4, angle_to_first * 10))

    env.update(steer_angle, 100, diff*10, False, False)
    reward = env.get_reward()
    #print(reward)
    
    frame += 1

env.save()