import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from actor import Actor
from critic import Critic
from replayBuffer import ReplayBuffer
from noise import OUActionNoise
import time

class Agent(object):
    def __init__(self, n_states, n_actions, 
                 lr_actor, lr_critic, 
                 tau, gamma,
                 mem_size, 
                 actor_l1_size, actor_l2_size,
                 critic_l1_size, critic_l2_size, 
                 batch_size):

        self.gamma = gamma
        self.tau = tau
        self.memory = ReplayBuffer(mem_size, n_states, n_actions)
        self.batch_size = batch_size


        self.actor = Actor(lr_actor, n_states, n_actions, actor_l1_size, actor_l2_size)
        self.critic = Critic(lr_critic, n_states, n_actions, critic_l1_size, critic_l2_size)

        self.target_actor = Actor(lr_actor, n_states, n_actions, actor_l1_size, actor_l2_size)
        self.target_critic = Critic(lr_critic, n_states, n_actions, critic_l1_size, critic_l2_size)

        self.noise = OUActionNoise(mu=np.zeros(n_actions))

        self.update_network_parameters(tau=1)

    def choose_action(self, observation):
        self.actor.eval()
        observation = torch.tensor(observation, dtype=torch.float).to(self.actor.device)
        mu = self.actor.forward(observation).to(self.actor.device)

        # add noise to action - for exploration
        mu_prime = mu + torch.tensor(self.noise(),
                                 dtype=torch.float).to(self.actor.device)
        self.actor.train()

        return mu_prime.cpu().detach().numpy()


    def remember(self, state, action, reward, new_state, done):
        self.memory.push(state, action, reward, new_state, done)

    def learn(self):
        if self.memory.idx_last < self.batch_size:
            # not enough data in replay buffer
            return

        # select random events
        state, action, reward, new_state, done = self.memory.sample_buffer(self.batch_size)

        reward = torch.tensor(reward, dtype=torch.float).to(self.critic.device)
        done = torch.tensor(done).to(self.critic.device)
        new_state = torch.tensor(new_state, dtype=torch.float).to(self.critic.device)
        action = torch.tensor(action, dtype=torch.float).to(self.critic.device)
        state = torch.tensor(state, dtype=torch.float).to(self.critic.device)

        self.target_actor.eval()
        self.target_critic.eval()
        self.critic.eval()
        target_actions = self.target_actor.forward(new_state)
        critic_value_ = self.target_critic.forward(new_state, target_actions)
        critic_value = self.critic.forward(state, action)

        target = []
        for j in range(self.batch_size):
            target.append(reward[j] + self.gamma*critic_value_[j]*done[j])
        target = torch.tensor(target).to(self.critic.device)
        target = target.view(self.batch_size, 1)

        self.critic.train()
        self.critic.optimizer.zero_grad()
        critic_loss = F.mse_loss(target, critic_value)
        critic_loss.backward()
        self.critic.optimizer.step()

        self.critic.eval()
        self.actor.optimizer.zero_grad()
        mu = self.actor.forward(state)
        self.actor.train()
        actor_loss = -self.critic.forward(state, mu)
        actor_loss = torch.mean(actor_loss)
        actor_loss.backward()
        self.actor.optimizer.step()

        self.update_network_parameters()

    def update_network_parameters(self, tau=None):
        if tau is None:
            tau = self.tau

        actor_params = self.actor.named_parameters()
        critic_params = self.critic.named_parameters()
        target_actor_params = self.target_actor.named_parameters()
        target_critic_params = self.target_critic.named_parameters()

        critic_state_dict = dict(critic_params)
        actor_state_dict = dict(actor_params)
        target_critic_dict = dict(target_critic_params)
        target_actor_dict = dict(target_actor_params)

        for name in critic_state_dict:
            critic_state_dict[name] = tau*critic_state_dict[name].clone() + \
                                      (1-tau)*target_critic_dict[name].clone()

        self.target_critic.load_state_dict(critic_state_dict)

        for name in actor_state_dict:
            actor_state_dict[name] = tau*actor_state_dict[name].clone() + \
                                      (1-tau)*target_actor_dict[name].clone()
        self.target_actor.load_state_dict(actor_state_dict)

    def save_models(self):
        timestamp = time.strftime("%Y%m%d-%H%M%S")

        self.actor.save("actor_" + timestamp)
        self.target_actor.save("target_actor_" + timestamp)
        self.critic.save("critic_" + timestamp)
        self.target_critic.save("target_critic_" + timestamp)

    def load_models(self, fn_actor, fn_target_actor, fn_critic, fn_target_critic):
        self.actor.load_checkpoint(fn_actor)
        self.target_actor.load_checkpoint(fn_target_actor)
        self.critic.load_checkpoint(fn_critic)
        self.target_critic.load_checkpoint(fn_target_critic)