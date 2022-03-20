#importing required libraries
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import glob
import yaml
import math

# parsing through the config file to read constants
with open("../configs/constants.yml", 'r') as config_file:
    constants = yaml.full_load(config_file)
    frequency = constants["RATE_HZ"]
#function to plot joint values
def plot_joint_values():
    path = 'data' # using the folder name
    file_name = glob.glob("../"+path + "/*.csv")
    #function for filtering friction torque
    def vector_filtering(x, y):
        A = np.hstack((x[np.newaxis].T, np.ones((len(x), 1))))
        b = y[np.newaxis].T
        U, s, Vt = np.linalg.svd(A)
        s_inv = np.array(s)

        for i in range(len(s_inv)):
            if abs(s_inv[i] - 0.) > 1e-5:
                s_inv[i] = 1. / s_inv[i]

        S_inv = np.zeros((Vt.shape[0], U.shape[1]))
        np.fill_diagonal(S_inv, s_inv)
        A_pinv = Vt.T.dot(S_inv).dot(U.T)
        X = A_pinv.dot(b)
        y_filter = X[0] * x + X[1]
        return (x,y,y_filter)
    #function for parsing through dymanic data and plotting joint values
    for file in range(len(file_name)):
        jnt_velocity=[]
        friction_torque=[]
        time=[]
        counter=0
        content = genfromtxt(file_name[file], delimiter=' ')
        _ , rhs = file_name[file].split("_", 1)
        for iteration in content[:,2]:
            jnt_velocity.append(iteration)
            counter=counter+1
            time.append(counter)
        for iteration in content[:,6]:
            if math.isinf(iteration):
                friction_torque.append(0)
            else:
                friction_torque.append(iteration)
        for iteration in range(len(time)):
            time[iteration]=time[iteration]/frequency
        
        fig, (ax1,ax2) = plt.subplots(2)
        fig.set_figwidth(40)
        fig.set_figheight(20)
        fig.suptitle('Joint vectors')

        ax1.plot(time, jnt_velocity, color='b', label='Joint Velocity')
        ax1.legend()
        ax1.grid()
        ax1.set_xlabel("Time (sec)")
        ax1.set_ylabel("[rad/sec]")
        
        x,y,yfilter = vector_filtering(np.array(time), np.array(friction_torque))
        ax2.plot(time, friction_torque, color='g', label='Friction Torque')
        ax2.plot(x, yfilter, color='r', label='Friction_Torque_Filtered')
        ax2.legend()
        ax2.grid()
        ax2.set_xlabel("Time (sec)")
        ax2.set_ylabel("[Nm]")

        plt.savefig('../plots/Joint_values_at_'+rhs[0:4]+'_rad_per_sec_velocity.png')
