import numpy as np
import gym
import gazebo_pusher
from buffer import ReplayBuffer
from dqn import DQN
import time
import matplotlib.pyplot as plt
import random
import os
from datetime import datetime
import pickle
import warnings

if __name__ == '__main__':
    warnings.filterwarnings("ignore")
    env = gym.make('Planar5DoF-v0')
    actions = range(50)
    memory_capacity = 20000
    memory = ReplayBuffer(memory_capacity)
    policy_net = DQN(10, 50)
    n_episodes = 4000
    n_steps = 150
    all_rewards = []
    epsilon = 0.99
    decay = 0.999
    model = policy_net.model
    sample_size = 2000
    gamma = 0.8
    x_finals = []
    target_update_period = 15
    for e in range(n_episodes):
        total_reward = 0
        if (e == 0):
            time.sleep(3) # to load all controllers first
        print("New episode #" + str(e))
        state = np.array(env.reset())

        if epsilon > 0.1:
            epsilon *= decay
        for i in range(n_steps):
            # print(state)
            q_values = model.predict(np.expand_dims(state.reshape(1,10), axis=0))
            # print(q_values)

            if (random.random() < epsilon):
                action = np.random.choice(actions)
            else:
                action = np.argmax(q_values)
            nextState, reward, done, x = env.step(action)
            nextState = np.array(nextState)
            total_reward += reward
            memory.add(state, action, reward, done, nextState)
            # print(state)
            if memory.len() > sample_size:
                policy_net.replay(memory, gamma, sample_size)
            if done:
                break
            else:
                state = nextState
        if e % target_update_period == 0:
            policy_net.target_update()
        x_finals.append(x - 2.13)
        all_rewards.append(total_reward)
        print("Total number of steps: %2d; Total reward: %2.2f; Epsilon: %3.2f; Final x: %3.2f" % (i + 1, total_reward, epsilon, x-2.13))
    model.save('/home/arms/dqn3.h5')
    t = np.arange(n_episodes)
    today = datetime.now()
    path = os.getcwd()
    name = str(today)
    try:
        os.mkdir(name)
    except OSError:
        print ("Creation of the directory %s failed" % path)
    else:
        print ("Successfully created the directory %s " % path)
    outfile_rewards = os.path.join(path, name, "rewards")
    outfile_x = os.path.join(path, name, "xs")
    with open(outfile_rewards, 'wb') as fp:
        pickle.dump(all_rewards, fp)

    with open(outfile_x, 'wb') as fp:
        pickle.dump(x_finals, fp)
    fig, axs = plt.subplots(2)
    axs[0].plot(t, all_rewards)
    axs[1].plot(t, x_finals)

    plt.show()
