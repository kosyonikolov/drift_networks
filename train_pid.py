from environment import Environment
import numpy as np
import sys
import torch
import os
from agent import Agent

# 10 vectors in polar coordinates
N_STATES = 2 * 10
# brake
N_ACTIONS = 1

ACTOR_L1 = 256
ACTOR_L2 = 256
CRITIC_L1 = 256
CRITIC_L2 = 256

LR_ACTOR  = 0.000025
LR_CRITIC = 0.00025

REPLAY_MEMORY_SIZE = 20000000
BATCH_SIZE = 256

MAX_ITERS = 60 * 30
MAX_EPISODES = 200000
DUMP_PERIOD  = 150

MAX_STEERING_ANGLE = 3.14 / 4

# load track
track = np.loadtxt(sys.argv[1])
env = Environment(track, initial_velocity=15)

# create agent
agent = Agent(N_STATES, N_ACTIONS, LR_ACTOR, LR_CRITIC, 0.0005, 0.99, REPLAY_MEMORY_SIZE, \
             ACTOR_L1, ACTOR_L2, CRITIC_L1, CRITIC_L2, BATCH_SIZE)

if len(sys.argv) == 6:
    fn_actor = sys.argv[2]
    fn_critic = sys.argv[3]
    fn_target_actor = sys.argv[4]
    fn_target_critic = sys.argv[5]

    agent.load_models(fn_actor, fn_target_actor, fn_critic, fn_target_critic)


rewards_log = open("dump/rewards.txt", "w")

for i in range(MAX_EPISODES):

    should_dump = i % DUMP_PERIOD == 0

    env.reset(should_dump)
    done = False
    total_reward = 0

    iter = 0

    while not done:
        state, _ = env.get_state()
        angle_to_first = state[2][1]
        state = np.array(state)
        in_state_nn = state.flatten()

        action = agent.choose_action(in_state_nn)

        # map from -1..1 to proper inputs

        steer = min(3.14 / 4, max(-3.14 / 4, angle_to_first * 2.8))
        gas   = 90
        brake = action[0] * 50 + 50

        env.update(steer, gas, brake, False, False)

        reward = env.get_reward()

        state, done = env.get_state()
        done |= iter > MAX_ITERS
        
        state = np.array(state)
        out_state_nn = state.flatten()

        agent.remember(in_state_nn, action, reward, out_state_nn, 1 if done else 0)
        agent.learn()

        total_reward += reward
        iter += 1
        

    print("ep {0}, reward = {1:.2f}".format(i, total_reward))
    rewards_log.write("{0:.2f}\n".format(total_reward))
    rewards_log.flush()

    if should_dump:
        agent.save_models()
        env.save()