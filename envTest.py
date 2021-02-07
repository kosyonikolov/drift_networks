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

total_reward = 0

while not done and frame < max_frames:
    state, done = env.get_state()
    angle_to_first = state[2][1]

    #print(angle_to_first)

    steer_angle = min(3.14 / 4, max(-3.14 / 4, angle_to_first * 2.8))

    env.update(steer_angle, 50, 0, False, False)
    reward = env.get_reward()
    total_reward += reward
    #print(reward)
    
    frame += 1

print("{0:.2f}".format(total_reward))
env.save()