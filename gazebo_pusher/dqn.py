import numpy as np
import gym
import gazebo_pusher
import tensorflow as tf
import keras
from keras import layers
from keras.models import Sequential
from keras.layers import Activation, Dense
from keras.optimizers import Adam
import buffer
import random


class DQN:
    def __init__(self, n_input, n_output):
        self.input_shape = n_input
        self.output_shape = n_output
        self.model = self.create_model()
        self.target_model = self.create_model()

    def create_model(self):
        inputs = layers.Input(shape=(1, self.input_shape))
        layer1 = Dense(32, activation='relu', kernel_initializer=keras.initializers.Zeros())(inputs)
        layer2 = Dense(64, activation='relu')(layer1)
        layer3 = Dense(256, activation='relu')(layer2)
        layer4 = Dense(1024, activation='relu')(layer3)
        layer5 = Dense(2048, activation='relu')(layer4)
        layer6 = Dense(512, activation='relu')(layer5)
        layer7 = Dense(128, activation='relu')(layer6)
        output = Dense(self.output_shape, activation='linear')(layer7)
        model = keras.Model(inputs = inputs, outputs = output)
        model.compile(loss='mse', optimizer = Adam(0.001))
        return model

    def update(self, state, q_values):
        y_pred = self.model.predict(np.expand_dims(state.reshape(1,self.input_shape), axis=0))
        self.model.fit(state.reshape(-1,1,self.input_shape), q_values, verbose = 0)

    def replay(self, memory, gamma, sample_size):
        states, actions, rewards, dones, nextStates = memory.sample(sample_size)
        states = np.asarray(states)
        dones = np.asarray(dones)
        nextStates = np.asarray(nextStates)
        q_values = self.model.predict(states.reshape(-1,1,self.input_shape)).reshape(-1,self.output_shape) # (16,1,2)
        # print(gamma)
        # print(q_values)
        # print("------------")
        # print(dones)
        # print("------------")
        # print(actions)
        # print("------------")
        q_nexts = self.model.predict(nextStates.reshape(-1,1,self.input_shape)).reshape(-1,self.output_shape)
        q_nexts_target = self.target_model.predict(nextStates.reshape(-1,1,self.input_shape)).reshape(-1,self.output_shape)
        for i in range(len(q_values)):
            if (dones[i]):
                q_values[i][actions[i]] = rewards[i]
            else:
                a = np.argmax(q_nexts)
                q_values[i][actions[i]] = rewards[i] + gamma * q_nexts_target[i][a]
        # print(q_values)
        # print("------------")
        # print(q_nexts)
        # print("------------")
        # print(q_nexts_target)
        # print("------------")
        # print(rewards)
        # print("------------")
        # print("+++++++++++++++")
        self.model.fit(states.reshape(-1,1,self.input_shape), q_values.reshape(-1,1,self.output_shape), epochs = 1, verbose = 0)

    def target_update(self):
        self.target_model.set_weights(self.model.get_weights())
