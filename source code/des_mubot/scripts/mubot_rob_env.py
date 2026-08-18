# -*- coding: utf-8 -*-
"""
Created on Mon Aug 17 12:50:38 2020
Update on 01/22/2021
@author: donghao Li
"""

import rospy
import rospkg
import time
import numpy as np
import math
import copy
#from cpg_function import CPG
import numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.srv import JointRequest
from gazebo_connection import GazeboConnection
from controllers_connection import ControllersConnection

from geometry_msgs.msg import Point
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from tf.transformations import euler_from_quaternion



class MubotEnv():

    def __init__(self):

        # Done
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('des_mubot')
        self.max_distance = rospy.get_param('/mubot/max_distance')
        self.max_sim_time = rospy.get_param('/mubot/max_sim_time')        
#        self.total_timestep = int(self.max_sim_time*1000.0)
        self.total_timestep = int(self.max_sim_time*4000.0)
        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        self.gazebo.unpauseSim()
        self.gazebo.pauseSim()
        self.reset

    def close(self):
        self.reset
        self.gazebo.pauseSim()


    def step(self, useless):
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        done = False
        self.gazebo.unpauseSim()
        while not done:
            now = rospy.get_time()
            if now >= self.max_sim_time-0.1:
                done = True
            else:
                done = False
        obs = self._get_obs()
        self.gazebo.pauseSim()
        velocity_read = np.loadtxt(open("./result/DD/velocity.csv","rb"),delimiter=",",skiprows=0)
#        if len(velocity_read) >=2000:
#            self.vel_all = velocity_read[0:2000,:]
        if len(velocity_read) >=8000:
            self.vel_all = velocity_read[0:8000,:]
        else:
            self.vel_all = velocity_read.copy()
        if len(velocity_read)!=0:
            vel_mean = np.mean(self.vel_all,axis=0)
            mean_x_vel = -vel_mean[0]
            mean_y_vel = +vel_mean[1]
            reward = mean_x_vel
        else:
            reward = 0.0
        simplified_obs = self.convert_obs_to_state(obs,mean_x_vel,mean_y_vel)
        info = {}
        self.gazebo.pauseSim()
        return simplified_obs, reward, done, info


    def reset(self):
        self.gazebo.unpauseSim()
        for i in range(10):
            reset = False
            self.gazebo.resetSim()
            while reset is not True:
                now = rospy.get_time()*1000+0.1
                if now >45:
                    reset = True
        self.gazebo.resetSim()
        self.gazebo.resetSim()
        self.gazebo.pauseSim()
        self.init_env_variables()
        obs = self._get_obs()
        simplified_obs = self.convert_obs_to_state(obs,0.0,0.0)
        self.vel_all =[]
        return simplified_obs


    def init_env_variables(self):
        self.total_distance_moved = 0.0
        self.current_x_vel = 0.0
        self.vol_traj = np.array([])
        self.vol = np.array([])
        self.current = np.zeros(6)
        self.effort = np.zeros(6)
        self.power = np.zeros(6)
        self.reward_power = 0.0
        self.reward_power_prev = 0.0
        self.xvels = []
        self.yvels = []
        self.pows = []
        self.counter = 0


    def _get_obs(self):
        x_distance = 0
        y_distance = 0
        x_linear_speed = 0
        y_linear_speed = 0
        sim_time = rospy.get_time() # double in second
        mubot_observations = [
            sim_time,
            x_distance,
            x_linear_speed,
            y_distance,
            y_linear_speed,
        ]
        return mubot_observations


    def check_all_sensors_ready(self):
#        self.check_joint_states_ready()
#        self.check_odom_ready()
        rospy.logdebug("ALL SENSORS READY")

    def convert_obs_to_state(self,observations,x,y):
#        state_converted = np.zeros(10)
        #state_converted=[sim_time, x_velocity, x_distance, y_velocity, y_distance]
        state_converted = observations
        state_converted[1] = x
        state_converted[3] = y
        return state_converted
