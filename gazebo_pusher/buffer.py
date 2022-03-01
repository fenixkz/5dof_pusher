from collections import deque
import numpy as np
import random



class ReplayBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = deque(maxlen= size)

    def add(self, state, action, reward, done, nextState):
        exp = (state, action, reward, done, nextState)
        self.buffer.append(exp)

    def sample(self, sample_size):
        state_batch = []
        action_batch = []
        reward_batch = []
        done_batch = []
        nextState_batch = []

        batch = random.sample(self.buffer, sample_size)
        for exp in batch:
            s, a, r, d, n = exp
            state_batch.append(s)
            action_batch.append(a)
            reward_batch.append(r)
            done_batch.append(d)
            nextState_batch.append(n)
        return state_batch, action_batch, reward_batch, done_batch, nextState_batch

    def len(self):
        return len(self.buffer)

    def print_memory(self):
        print(self.buffer)
