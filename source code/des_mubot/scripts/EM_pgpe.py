# This Python file uses the following encoding: utf-8

# if__name__ == "__main__":
#     pass
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 26 2021
Update on
@author: donghao Li
"""

import numpy as np
import rospy
import math

class EMPGPE():
    def __init__(self, n_parmas,n_rollout,joint_number,mu,sigma,select):
        self.n_parmas = n_parmas # number of parameter
        self.n_rollout = n_rollout # number of rollout
        self.j_number = joint_number
        self.mu = mu
        self.sigma = sigma
        self.select = select
        self.sig_range = np.zeros(n_parmas)
        for i in range (joint_number-1):
            self.sig_range[i*2]   = 1E-2
            self.sig_range[i*2+1] = 1E-6
        self.sig_range[joint_number*2-2] = 1E-2
        self.sig_range[joint_number*2-1] = 1E-6

    def sample_parmas(self, mu, sigma):
        s_param = np.zeros([self.n_rollout,self.n_parmas])
        for i in range(self.n_parmas):
            s_param[:,i] = np.random.normal(mu[i],sigma[i],self.n_rollout)
        return s_param

    def learn(self, thetas, reward, pre_mu, pre_sigma):
#        reward = np.array()
        sorted_index = reward.argsort()[::-1][:self.select]
        s_reward = reward[sorted_index]+0.1
        s_thetas = thetas[sorted_index]
#        Update mu and sigma
        new_mu = sum(s_reward[:,None]*s_thetas)/sum(s_reward)
        for i in range(self.j_number-1):
            new_mu[2*i+1] = new_mu[2*i+1]%1
        new_sigma = np.sqrt( sum( s_reward[:,None]*np.square(s_thetas-new_mu[None,:]) )/sum(s_reward) )

#        Constrain
        new_sigma = np.clip(new_sigma,0.8*pre_sigma,None)
        new_sigma = np.clip(new_sigma,self.sig_range,None)

#        Transport
        delta_mu = new_mu-pre_mu
        self.mu = new_mu
        self.sigma = new_sigma
        return new_mu, new_sigma, delta_mu



