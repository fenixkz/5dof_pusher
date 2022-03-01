import numpy as np
from dqn import DQN
import keras
import time
import gym
import gazebo_pusher
model = keras.models.load_model('/home/arms/dqn_0.h5')
env = gym.make('Planar5DoF-v0')
actions = range(10)

done = False
state = np.array(env.reset())
while not done:
    action = np.argmax(model.predict(np.expand_dims(state.reshape(1,10), axis=0)))
    # print(state)
    # action = int(input())
    nextState, reward, done, _ = env.step(action)
    nextState = np.array(nextState)
    state = nextState
