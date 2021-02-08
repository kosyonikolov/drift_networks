import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

class Actor(nn.Module):
    def __init__(self, learning_rate, n_states, n_actions, fc1_dims, fc2_dims,
                 dump_dir='dump/models/'):
        
        super(Actor, self).__init__()
        self.n_states = n_states
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions
        self.dump_dir = dump_dir

        self.fc1 = nn.Linear(n_states, self.fc1_dims)
        
        f1 = 1./np.sqrt(self.fc1.weight.data.size()[0])
        torch.nn.init.uniform_(self.fc1.weight.data, -f1, f1)
        torch.nn.init.uniform_(self.fc1.bias.data, -f1, f1)


        self.fc2 = nn.Linear(self.fc1_dims, self.fc2_dims)
        f2 = 1./np.sqrt(self.fc2.weight.data.size()[0])
        torch.nn.init.uniform_(self.fc2.weight.data, -f2, f2)
        torch.nn.init.uniform_(self.fc2.bias.data, -f2, f2)

        f3 = 0.003

        # final layer
        self.mu = nn.Linear(self.fc2_dims, self.n_actions)
        torch.nn.init.uniform_(self.mu.weight.data, -f3, f3)
        torch.nn.init.uniform_(self.mu.bias.data, -f3, f3)

        self.optimizer = torch.optim.Adam(self.parameters(), lr=learning_rate)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.to(self.device)

    def forward(self, state):
        x = self.fc1(state)
        x = F.relu(x)
        x = self.fc2(x)
        x = F.relu(x)
        x = torch.tanh(self.mu(x))

        return x

    def save(self, name):
        print("(actor) saving ", name)
        torch.save(self.state_dict(), self.dump_dir + name + ".network")

    def load_checkpoint(self, name):
        print("(actor) loading ", name)
        self.load_state_dict(torch.load(name))