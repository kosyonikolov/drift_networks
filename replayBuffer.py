import numpy as np

class ReplayBuffer(object):
    def __init__(self, max_size, state_shape, n_actions):
        self.size = max_size
        self.idx_last = 0

        self.states = np.zeros((self.size, *state_shape))
        self.new_states = np.zeros((self.size, *state_shape))
        self.actions = np.zeros((self.size, n_actions))
        self.rewards = np.zeros(self.size)

        # 0 if next state is terminal, 1 otherwise
        self.has_nexts = np.zeros(self.size, dtype=np.float32)

    def push(self, state, action, reward, next_state, done):
        idx = self.idx_last % self.size
        self.idx_last += 1

        self.states[idx] = state
        self.new_states[idx] = next_state
        self.actions[idx] = action
        self.rewards[idx] = reward
        self.has_nexts[idx] = 1 - done
        

    def sample_buffer(self, batch_size):
        count = min(self.idx_last, self.size)

        selection = np.random.choice(count, batch_size)

        states = self.states[selection]
        actions = self.actions[selection]
        rewards = self.rewards[selection]
        states_ = self.new_states[selection]
        has_next = self.has_nexts[selection]

        return states, actions, rewards, states_, has_next