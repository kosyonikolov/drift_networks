from environment import Environment
import numpy as np
import sys

# load track
track = np.loadtxt(sys.argv[1])

env = Environment(track)

env.reset(True)

done = False

while not done:
    env.update(0, 100, 0, False, False)
    _, done = env.get_state()
    reward = env.get_reward()
    print(reward)

env.save()