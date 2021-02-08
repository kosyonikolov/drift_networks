import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

class Critic(nn.Module):
    def __init__(self, learning_rate, n_states, n_actions, fc1_dims, fc2_dims, 
                 dump_dir='dump/models/'):

        super(Critic, self).__init__()
        self.n_states = n_states
        self.n_actions = n_actions

        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        
        self.fc1 = nn.Linear(self.n_states, self.fc1_dims)
        f1 = 1.0  / np.sqrt(self.fc1.weight.data.size()[0])
        torch.nn.init.uniform_(self.fc1.weight.data, -f1, f1)
        torch.nn.init.uniform_(self.fc1.bias.data, -f1, f1)

        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        f2 = 1.0 / np.sqrt(self.fc2.weight.data.size()[0])
        torch.nn.init.uniform_(self.fc2.weight.data, -f2, f2)
        torch.nn.init.uniform_(self.fc2.bias.data, -f2, f2)

        self.fc1_actions = nn.Linear(self.n_actions, self.fc2_dims)
        
        # final layer
        self.q = nn.Linear(self.fc2_dims, 1)
        f3 = 0.003
        torch.nn.init.uniform_(self.q.weight.data, -f3, f3)
        torch.nn.init.uniform_(self.q.bias.data, -f3, f3)

        self.optimizer = torch.optim.Adam(self.parameters(), lr=learning_rate)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.to(self.device)

        self.dump_dir = dump_dir

    def forward(self, state, action):

        state_value = self.fc1(state)
        state_value = F.relu(state_value)
        state_value = self.fc2(state_value)

        action_value = F.relu(self.fc1_actions(action))
        state_action_value = F.relu(torch.add(state_value, action_value))
        state_action_value = self.q(state_action_value)

        return state_action_value

    def save(self, name):
        print("(critic) saving ", name)
        torch.save(self.state_dict(), self.dump_dir + name + ".network")

    def load_checkpoint(self, name):
        print("(critic) loading ", name)
        self.load_state_dict(torch.load(name))