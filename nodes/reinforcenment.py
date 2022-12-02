#!/usr/bin/env python
import numpy as np
import rospy
import random
from collections import deque
import tensorflow as tf
from tensorflow import keras

from keras.models import Sequential, load_model
from keras.optimizers import Adam,RMSprop
from keras.layers import Dense, Dropout, Activation
from keras.callbacks import History, TerminateOnNaN, EarlyStopping, ReduceLROnPlateau
import os
import sys
import json
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))


class ReinforcementNetwork(object):
    """
    Algorithm DDRLE-GE
    """
    def __init__(self,state_size,action_size,number_episode,load,rank_cloud):
        # self.dirPath            = os.path.dirname(os.path.realpath(__file__))
        # self.dirPath            = self.dirPath.replace('multi_robot/src/nodes', 'multi_robot/src/save_model/environment_')
        self.dirPath            =  "/home/mcg/catkin_ws/src/multi_robot/save_model/environment"
        self.state_size         = state_size
        self.action_size        = action_size
        self.load_episode       = number_episode
        self.discount_factor    = 0.99
        self.learning_rate      = 0.00030 #0.00025
        self.batch_size         = 96#128s
        self.train_start        = 96#128
        self.Pa                 = 0
        self.Pbest              = 0.01
        self.Pbest_max          = 0.95
        self.size_layer_1       = 512#1000
        self.size_layer_2       = 512#512 256
        self.reward_max         = 0
        self.tau                = 0.3
        self.target_value       = 0
        self.dropout            = 0.2
        self.lim_q_s            = 0.95
        self.lim_q_i            = 0.25
        self.target_update      = 2000
        self.start_or           = 0.186
        self.lim_train          = 0.186#0.25#0.36
        self.memory_D           = deque(maxlen=100000)
        self.memory_GT          = deque(maxlen=100000)
        self.memory_EPS         = deque(maxlen=100000)
        self.load_model         = load
        self.loss               = 'mse'
        self.activation_output  = 'linear'
        self.activation_layer   = 'relu'
        self.kernel_initializador = 'lecun_uniform'
        self.q_model              = self.q_network()
        self.target_model         = self.target_network()
        self.normal_process       = 0.186
        self.increase_factor    = 0.97#70.975 env1 for paper
        self.load_epidose       = 0
        self.rank_cloud         = rank_cloud

        if self.load_model:
            self.q_model.set_weights(load_model(self.dirPath+str(self.load_episode)+'_'+str(self.rank_cloud)+'_q_model'+".h5").get_weights())
            self.target_model.set_weights(load_model(self.dirPath+str(self.load_episode)+'_'+str(self.rank_cloud)+'_target_model'+".h5").get_weights())
            self.Pa,self.Pbest= self.load_mode()
            print("PA NUBE", self.Pa)
        else:
            self.q_model              = self.q_network()
            self.target_model         = self.target_network()

    def load_mode(self):
        with open(self.dirPath+str(self.load_episode)+'.json') as outfile:
            param = json.load(outfile)
            Pa = param.get('Pa')
            Pbest = param.get('Pbest')
        return  Pa, Pbest

    def q_network(self):
        '''
        In this network we evaluate the action of the q_network and predict the following value of Q(s',a)
        '''
        q_model = Sequential()
        q_model.add(Dense(self.size_layer_1, input_shape=(self.state_size,), activation= self.activation_layer, kernel_initializer = self.kernel_initializador))
        q_model.add(Dense(self.size_layer_2, activation= self.activation_layer, kernel_initializer=self.kernel_initializador))
        q_model.add(Dropout(self.dropout))
        q_model.add(Dense(self.action_size, kernel_initializer=self.kernel_initializador))
        q_model.add(Activation(self.activation_output))
        q_model.compile(loss=self.loss, optimizer=RMSprop(lr=self.learning_rate, rho=0.9,  epsilon=1e-08, decay=0.0), metrics=['acc'])
        return q_model

    def target_network(self):
        '''
        In this network we evaluate the action of the q_network and predict the following value of Q(s',a)
        '''
        target_model = Sequential()
        target_model.add(Dense(self.size_layer_1, input_shape=(self.state_size,), activation= self.activation_layer, kernel_initializer = self.kernel_initializador))
        target_model.add(Dense(self.size_layer_2, activation= self.activation_layer, kernel_initializer=self.kernel_initializador))
        target_model.add(Dropout(self.dropout))
        target_model.add(Dense(self.action_size, kernel_initializer=self.kernel_initializador))
        target_model.add(Activation(self.activation_output))
        target_model.compile(loss=self.loss, optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-08, decay=0.0), metrics=['acc'])
        return target_model

    def get_Qvalue(self, reward, next_target, done):
        if done:
            return reward
        else:
            return reward + self.discount_factor * next_target

    def update_target_cloud(self):
        q_model_theta = self.q_model.get_weights()
        target_model_theta = self.target_model.get_weights()
        counter = 0
        for q_weight, target_weight in zip(q_model_theta, target_model_theta):
            target_weight = target_weight *(1-self.tau) + q_weight*self.tau
            target_model_theta[counter] = target_weight
            counter += 1
        # rospy.loginfo("UPDATE TARGET NETWORK CLOUD")
        self.target_model.set_weights(target_model_theta)

    def merge_target_cloud(self,q_model_theta,target_model_theta):
        target_model_theta = self.q_model.get_weights()
        counter = 0
        for q_weight, target_weight in zip(q_model_theta, target_model_theta):
            target_weight = target_weight *(1-self.tau) + q_weight*self.tau
            target_model_theta[counter] = target_weight
            counter += 1
        self.q_model.set_weights(target_model_theta)

        q_model_theta_to = target_model_theta
        target_model_theta_to = self.target_model.get_weights()
        counter_t_to = 0
        for q_weight_to, target_weight_to in zip(q_model_theta_to, target_model_theta_to):
            target_weight_to = target_weight_to *(1-self.tau) + q_weight_to*self.tau
            target_model_theta_to[counter_t_to] = target_weight_to
            counter_t_to += 1
        self.target_model.set_weights(target_model_theta_to)

    def update_target_network(self,q_model_theta,target_model_theta):
        # rospy.loginfo("UPDATE TARGET NETWORK")
        self.q_model.set_weights(q_model_theta)
        self.target_model.set_weights(target_model_theta)


    def get_Pa(self):
        '''
        Calculates the probability by "Semi-Uniform Distributed Exploration"
        Pbes=0 purely random exploration, Pbest=1 pure exploitation.
        '''
        self.Pa = self.Pbest + ((1.0-self.Pbest)/self.action_size)
        return self.Pa

    def get_action(self,state):
        '''
        Action is determined based on directed knowledge, hybrid knowledge
        or autonomous knowledge
        '''
        n2 = np.random.rand()
        n3 = np.random.rand()

        if self.Pa <= self.lim_q_i:
            self.q_value = np.zeros(self.action_size)
            action = None
            evolve_rule = True
            # print("rules1")

        elif self.lim_q_s>self.Pa >self.lim_q_i:
            if n3 > self.Pa:
                self.q_value = np.zeros(self.action_size)
                action = None
                evolve_rule = True
                # print("rules2")

            else:
                self.q_value = self.q_model.predict(state.reshape(1,len(state)))
                action = np.argmax(self.q_value[0][:5])
                evolve_rule = False
                # print("1 Best action-rule")

        else:
            if n2 <= self.Pa:
                self.q_value = self.q_model.predict(state.reshape(1,len(state)))
                action = np.argmax(self.q_value[0][:5])
                evolve_rule = False
                # print("1 Best action")

            else:
                self.q_value = self.q_model.predict(state.reshape(1, len(state)))
                q_value = self.q_value[0][:-1]
                mask=~(np.arange(self.action_size -1)==np.argmax(q_value))
                q_value=q_value[mask]
                q_value= q_value-min(q_value)
                q_value = q_value/sum(q_value)
                action = np.random.choice(np.arange(self.action_size-1)[mask],p=q_value)
                evolve_rule = False
                print("2 Best action")
        return action, evolve_rule

    def append_D(self, state, action, reward, next_state, done):
        '''
        Memory used to train the model
        '''
        self.memory_D.append((state, action, reward, next_state, done))

    def append_EPS(self, state,action, reward, next_state, done):
        '''
        Memory for each episode,
        '''
        self.memory_EPS.append((state,action, reward, next_state, done))
    def append_GT(self, state,action, reward, next_state, done):
        '''
        Memory for each episode,
        '''
        self.memory_GT.append((state,action, reward, next_state, done))

    def winning_state(self):
        '''
        When the robot reaches the target, the temporary memory is copied into the
        main memory depending on the average reward.
        '''
        all_rewards = map(lambda x: x[2],self.memory_EPS)
        reward_aver = np.mean(all_rewards)
        if reward_aver > self.reward_max:
            self.reward_max = reward_aver
            self.memory_D.extend(self.memory_EPS)
            self.memory_D.extend(self.memory_EPS)
            self.memory_D.extend(self.memory_EPS)
            self.memory_EPS.clear()
            rospy.loginfo("Winning State with reward_max !!!")
        else:
            self.memory_EPS.clear()
            rospy.loginfo("Normal Win !!!")

    def best_state(self):
        '''
        When the robot reaches the goal with the best time, the temporary
        memory is copied into the main memory depending on the best time.
        '''
        self.memory_GT.extend(self.memory_EPS)
        self.memory_D.extend(self.memory_EPS)
        self.memory_D.extend(self.memory_EPS)
        self.memory_D.extend(self.memory_EPS)
        self.memory_EPS.clear()
        rospy.loginfo("Great Time !!!")

    def increase_fact(self):
        if self.Pbest < self.Pbest_max:
            self.Pbest /= self.increase_factor
        elif self.Pbest > self.Pbest_max:
            self.Pbest = self.Pbest_max
            self.Pa = self.Pbest_max
        else:
            self.Pbest = self.Pbest
        self.get_Pa()

    def start_training(self):
        if len(self.memory_D) > (self.train_start):
            self.train_model()
            return True
        else:
            return False

    def save_model(self,rank,e):
        self.q_model.save(self.dirPath + str(e)+"_"+str(rank)+'_q_model' +'.h5')
        self.target_model.save(self.dirPath + str(e) +"_"+str(rank)+'_target_model'+'.h5')
        param_keys = ['Pa','Pbest']
        param_values = [self.Pa, self.Pbest]
        param_dictionary = dict(zip(param_keys, param_values))
        with open(self.dirPath + str(e) + '.json', 'w') as outfile:
            json.dump(param_dictionary, outfile)

    def experience_replay(self):
        '''
        Based on probability choose random samples or continuous samples with
        the best average rewards
        '''
        batch_save   = []
        max_rew_save = []
        for i in range(1):
            num_2 = random.randrange(0,len(self.memory_D)-int(self.batch_size/8.0))
            if len(self.memory_GT)>2:
                mini_batch1 = deque(np.array(self.memory_D)[num_2:num_2+int(self.batch_size/8.0)-2])
            else:
                mini_batch1 = deque(np.array(self.memory_D)[num_2:num_2+int(self.batch_size/8.0)])
            batch_save.append(mini_batch1)
            all_rewards = np.array(map(lambda x: x[2],mini_batch1))
            max_reward = np.sum(all_rewards)
            max_rew_save.append(max_reward)
        idx_max_ = np.argmax(max_rew_save)

        if len(self.memory_GT)>2:
            id_gt =random.sample(self.memory_GT, 2)
            id_ra =random.sample(self.memory_D,int(self.batch_size-(self.batch_size/8.0)))
            mini_batch = batch_save[idx_max_]
            mini_batch.extend(id_gt)
            mini_batch.extend(id_ra)
        else:
            id_ra =random.sample(self.memory_D,int(self.batch_size-(self.batch_size/8.0)))
            mini_batch = batch_save[idx_max_]
            mini_batch.extend(id_ra)
        return mini_batch


    def train_model(self):
        mini_batch =  self.experience_replay()
        X_batch = np.empty((0, self.state_size), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        for i in range(self.batch_size):
            states = np.array(mini_batch[i][0])
            actions = mini_batch[i][1]
            rewards = mini_batch[i][2]
            next_states = np.array(mini_batch[i][3])
            dones = mini_batch[i][4]
            q_value = self.q_model.predict(states.reshape(1, len(states)))
            self.q_value = q_value
            next_q_value = self.q_model.predict(next_states.reshape(1, len(next_states)))
            id_max_1 = np.argmax(next_q_value)

            next_target = self.target_model.predict(next_states.reshape(1, len(next_states)))
            next_t =next_target[0][id_max_1]
            self.target_value = next_t
            next_q_value = self.get_Qvalue(rewards, next_t, dones)

            X_batch = np.append(X_batch, np.array([states.copy()]), axis=0)
            Y_sample = q_value.copy()

            Y_sample[0][actions] = next_q_value
            Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

            if dones:
                X_batch = np.append(X_batch, np.array([next_states.copy()]), axis=0)
                Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)
        reduce_lr = ReduceLROnPlateau(monitor='loss', factor=0.2, patience=0, min_lr=0.0001)
        result = self.q_model.fit(X_batch, Y_batch, batch_size=self.batch_size, epochs=1, verbose=0,callbacks=[reduce_lr])
        rospy.loginfo("Finish Training")
