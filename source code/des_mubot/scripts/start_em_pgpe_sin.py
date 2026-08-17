#!/usr/bin/env python

from EM_pgpe import EMPGPE
from mubot_rob_env import MubotEnv
import numpy as np
import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
import random
import os
import time, shutil, datetime

if __name__ == '__main__':

    rospy.init_node('mubot_learning', anonymous=True, log_level=rospy.WARN)
    # Create the Gym environment
    env = MubotEnv()
    rospy.loginfo ( "Learning environment done")
    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('des_mubot')
#    outdir = pkg_path + '/training_results'
    outdir = '/home/donghao/result/data'
    input_dir = '/home/donghao/result/DD/joint_input.csv'
    indicator_dir = '/home/donghao/result/DD/indicator.csv'

    ### Loads parameters from the ROS param server
    n_episodes = rospy.get_param("/mubot/n_episodes")
    n_rollout = rospy.get_param("/mubot/n_rollout")
    n_trial = rospy.get_param("/mubot/n_trial")
    joint_number = rospy.get_param("/mubot/joint_number")
    select = rospy.get_param("/mubot/select")

    nparams = joint_number*2

    ep_params = np.zeros([n_rollout,nparams])


    record_mu = np.zeros((n_episodes+1,nparams))
    record_sigma = np.zeros((n_episodes+1,nparams))
    record_reward = np.zeros((n_episodes,n_rollout))
    record_obs = np.zeros((n_episodes,n_rollout*4))
    episode_mu = np.zeros(nparams)
    episode_sigma = np.zeros(nparams)
    delta_mu = np.zeros(nparams)

    empgpe = EMPGPE(nparams,n_rollout,joint_number,record_mu[0,:],record_sigma[0,:],select)

    overview = np.array([0,0,0])

    temp_input = np.zeros(nparams+1)

    # Starts the main training loop: the one about the episodes to do
    z = 0

    time.sleep(5.0)

    if os.path.exists(outdir):
        shutil.rmtree(outdir)

    while z < n_trial:
        # rospy.logerr(" Trial=>" + str(z+1))
        x= 0
        record_mu.fill(0)
        record_sigma.fill(0)
        record_reward.fill(0)
        for i in range(joint_number-1):
            record_mu[0,2*i]   = random.uniform(10,15)
            record_mu[0,2*i+1] = random.uniform(0,1)
        record_mu[0,joint_number*2-2] = random.uniform(10,15)
        record_mu[0,joint_number*2-1] = random.uniform(0,7)

        for i in range(joint_number-1):
            record_sigma[0,2*i]   = 5
            record_sigma[0,2*i+1] = 0.3
        record_sigma[0,joint_number*2-2] = 5
        record_sigma[0,joint_number*2-1] = 2.5

        if not os.path.exists(outdir+'/'+str(z+1)):
            os.makedirs(outdir+'/'+str(z+1))

        while x < n_episodes:
            ep_params = empgpe.sample_parmas(record_mu[x,:], record_sigma[x,:])
            y = 0

            while y < n_rollout:
                if x==n_episodes-1:
                    indicator = np.array([z+1,y])
                else:
                    indicator = np.array([0,y])

#######################################################################################################
                temp_input = np.concatenate((ep_params[y,:],[5.0E-3/4*(4-int(z/2))]))
######################################################################################################

                f_input=open(input_dir,'w')
                np.savetxt(f_input, np.reshape(temp_input,(1,-1)),delimiter=',',fmt='%.6e')
                f_input.close()
                f_indicator=open(indicator_dir,'w')
                np.savetxt(f_indicator, np.reshape(indicator,(1,-1)),fmt='%i %i')
                f_indicator.close()

                rospy.logwarn("## T: " + str(z+1) + " E: "+str(x+1)+" R: " + str(y+1))
                done = False
                observation = env.reset()
                observation, reward, done, info = env.step(y)
                record_reward[x,y] = reward
                y += 1

            mean_reward = np.mean(record_reward[x,:])
            std_reward  = np.std(record_reward[x,:])
            overview = np.array([mean_reward,std_reward,std_reward/mean_reward*100])
            episode_mu, episode_sigma, delta_mu= empgpe.learn(ep_params,record_reward[x,:],record_mu[x,:],record_sigma[x,:])
            record_mu[x+1,:] = episode_mu
            record_sigma[x+1,:] = episode_sigma
            if x==0:
                f_pa=open(outdir+'/'+str(z+1)+'/param.csv','w')
                np.savetxt(f_pa, np.reshape(np.hstack((record_mu[0,:], record_sigma[0,:])),(1,-1)),fmt ='%.6e,')
                np.savetxt(f_pa, np.reshape(np.hstack((episode_mu, episode_sigma)),(1,-1)),fmt ='%.6e,')
                f_pa.close()

                f_re=open(outdir+'/'+str(z+1)+'/reward.csv','w')
                np.savetxt(f_re, np.reshape(record_reward[x,:],(1,-1)),fmt ='%.4e,')
                f_re.close()

                f_roll=open(outdir+'/'+str(z+1)+'/rollout_param.csv','w')
                np.savetxt(f_roll, ep_params,fmt ='%.6e,')
                f_roll.close()

            else:
                f_pa=open(outdir+'/'+str(z+1)+'/param.csv','a')
                np.savetxt(f_pa, np.reshape(np.hstack((episode_mu, episode_sigma)),(1,-1)),fmt ='%.6e,')
                f_pa.close()

                f_re=open(outdir+'/'+str(z+1)+'/reward.csv','a')
                np.savetxt(f_re, np.reshape(record_reward[x,:],(1,-1)),fmt ='%.4e,')
                f_re.close()

                f_roll=open(outdir+'/'+str(z+1)+'/rollout_param.csv','a')
                np.savetxt(f_roll, ep_params,fmt ='%.6e,')
                f_roll.close()

            x += 1
        z += 1

    indicator = [0,0]
    f_indicator=open(indicator_dir,'w')
    np.savetxt(f_indicator, np.reshape(indicator,(1,-1)),fmt='%i %i')
    f_indicator.close()

    rospy.logwarn("Learning is over. Yay!")


    env.close()


