import numpy as np
import gym
import gazebo_pusher
import qlearn

import time
import matplotlib.pyplot as plt

if __name__ == '__main__':
    env = gym.make('Planar5DoF-v0')
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=0.1, gamma=0.9, epsilon=0.9)
    highest_reward = 0
    f = open("qtable.txt", "w+")
    fa = open("actions.txt", "w+")
    result = []
    episode = []
    act = list()
    n_episodes = 10
    n_steps = 5
    for x in range(n_episodes):
        act1 = list()
        total_reward = 0
        if (x == 0):
            time.sleep(3) # to load all controllers first
        print("New episode #" + str(x))
        state = env.reset()

        if qlearn.epsilon > 0.02:
            qlearn.epsilon *= 0.99
        print(qlearn.epsilon)
        for i in range(n_steps):
            print("New step "+str(i))
            action = qlearn.chooseAction(state)
            act1.append(action)
            new_state, reward, done, info = env.step(action)
            total_reward += reward
            print("States we were "+str(state)+" action we took "+str(action)+" state we occured "+\
            str(new_state)+" reward we obtained "+str(reward)+" and total reward is "+str(total_reward))
            if highest_reward < total_reward:
                highest_reward = total_reward
                act = act1
            qlearn.learn(state,action,reward,new_state)

            if (done):
                break
            else:
                state = new_state
        episode.append(x)
        result.append(total_reward)
        print("Episode #"+str(x)+" has ended, total reward is "+str(total_reward))
    fa.write(str(act))
    f.write(str(qlearn.q))
    f.close()
    fa.close()
    plt.plot(episode, result)
    plt.show()
