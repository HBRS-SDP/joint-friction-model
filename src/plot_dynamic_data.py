#importing required libraries
import numpy as np
from numpy import genfromtxt
import matplotlib.pyplot as plt
import glob

def plot_joint_values():
    path = 'data' # use your folder name
    file_name = glob.glob("../"+path + "/*.csv")
    velocity = np.array([0.1,0.3,0.5,0.9,1.3])
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
        y_fit = X[0] * x + X[1]
        return (x,y,y_fit)
    #function for parsing through dymanic data and plotting joint values
    for j in range(len(file_name)):
        jnt_velocity=[]
        friction_torque=[]
        nominal_vel=[]
        time=[]
        counter=0
        content = genfromtxt(file_name[j], delimiter=' ')

        for i in content[:,2]:
            jnt_velocity.append(i)
            counter=counter+1
            time.append(counter)
        for i in content[:,6]:
            friction_torque.append(i)
        for i in content[:,8]:
            nominal_vel.append(i)
        for k in range(len(time)):
            time[k]=time[k]/900

        fig, (ax1,ax2) = plt.subplots(2)
        fig.set_figwidth(40)
        fig.set_figheight(20)
        fig.suptitle('Joint vectors')

        ax1.plot(time, jnt_velocity, color='r', label='Joint Velocity')
        ax1.plot(time, nominal_vel, color='b', label='Nominal Velocity')
        ax1.legend()
        ax1.grid()
        ax1.set_xlabel("Time (sec)")
        ax1.set_ylabel("[rad/sec]")
        
        x,y,yfit = vector_filtering(np.array(time), np.array(friction_torque))
        ax2.plot(x, y, color='g', label='Friction Torque')
        ax2.plot(x, yfit, color='r', label='Friction_Torque_Filtered')
        ax2.legend()
        ax2.grid()
        ax2.set_xlabel("Time (sec)")
        ax2.set_ylabel("[Nm]")

        plt.savefig('../plots/Joint_values_at_'+str(velocity[j])+'_rad_per_sec_velocity.png')
