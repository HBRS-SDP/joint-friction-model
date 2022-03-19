# Important notes

# 1 N = 1 (Kg * m)/s^2
# F = M * a                      FORCE
# I = m * r^2  (Kg * m^2)        INERTIA
# tau = I * a_ang                RELATION BETWEEN INERTIA AND ANGULAR ACCELERATION
# N * m = Kg * m^2 * rad / s^2 = (Kg * m^2)/s^2 

import numpy as np
from numpy import genfromtxt
import sympy as sp
import matplotlib.pyplot as plt
import csv
from typing import Tuple, Sequence, Dict
from functools import reduce
import random
import math 
import copy
import sys
import glob
import yaml
import pandas as pd

def get_breakaway_friction():
    path = './data/static_data/static_torque_breakaway_point.csv'
    rows = []
    data = pd.read_csv(path, sep = " ", header = None)
    Fc = np.mean(data[0])
    return Fc

def get_inertia():
    with open("./configs/constants.yml", "r") as file:
        documents = yaml.full_load(file)
        inertia = documents["joint_inertia"][documents["test_joint"]]
        return inertia
        
def fit_line(x, y):
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
    return (x, y, y_fit)

#FUNCTION TO DEFINE THE PARAMETERS AS SYMBOLS
def symbolic_poly(poly_params, Fs, v):
    return ((poly_params[0] + (Fs - poly_params[0]) * sp.exp(-(v/poly_params[1])**2))*np.sign(v) + poly_params[2]*v)

#FUNCTION TO CALCULATE THE ERROR OF THE ORIGINAL FUNCTION AND THE FUNCTION WITH NEW PARAMETERS
def error(poly_params, xs, ys, Fs):
    aux = []
    for i in range(len(xs)):
        aux.append((symbolic_poly(poly_params, Fs, xs[i])-ys[i])**2)
    return sum(aux)/len(aux)

#FUNCTION TO FIT THE PARAMETERS WITH THE ITERATIVE NEWTON METHOD
def newton(poly_params, xs, ys, param_value_guess, Fs, max_iter: int=100, epsilon: float=0.01) -> Tuple[np.ndarray, int, float]:
    params = np.array(param_value_guess)
    params_final = np.array(param_value_guess)
    final_error = 1e10    
    errorf = error(poly_params, xs, ys, Fs)
    X = sp.Matrix([errorf])
    Y = sp.Matrix(poly_params)
    myjac = X.jacobian(Y)
    xcurr = sp.Matrix(params)
    param_dict = dict(zip(poly_params, params))
    least_error = 1e10
    for iteration in range(max_iter):
        myjaceval = myjac.evalf(subs=param_dict)
        Xeval = X.evalf(subs=param_dict)
        xcurr = xcurr - myjaceval.pinv() @ Xeval
        for i,elem in enumerate(poly_params):
            param_dict[elem] = abs(xcurr[i])
        final_error = error(poly_params, xs, ys, Fs).subs(param_dict)
        if final_error <= least_error:
            least_error = final_error
            params_final = np.array(list(param_dict.values()),dtype=float)
        if final_error <= epsilon:
            break

    return (params_final, least_error)

#FUNCTION TO PLOT MODELS
def show_model(y_fit, y, x, xlabel, ylabel):
    plt.figure(figsize=(10, 10))
    plt.plot(x,y_fit)
    plt.grid(color='b', linestyle='-', linewidth=0.1)
    plt.scatter(x,y, color='r', linewidth=1)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.show()

def sim_mass_with_ramp_force_input(t, I, Fs, Fc, sigma_0, sigma_1, sigma_2, vs, force_rate, ts):
    zdot   = 0.0                    # z_dot
    qdot_2 = 0.0                    #x_dot_dot
    qdot_1 = 0.0                    #x_dot , q_2
    q_3    = 0.0                    # z
    q_1    = 0.0                    # x
    v      = []
    f      = []
    
    for i in t:
        zdot = qdot_1 - ((q_3 * abs(qdot_1) * sigma_0) / (Fc + (Fs - Fc) * np.exp(-(qdot_1/vs)**2)))
        F = sigma_0 * q_3 + sigma_1 * zdot + sigma_2 * qdot_1
        f.append(F)
        v.append(qdot_1)
        u = force_rate * i     # % ramped-up force input
        qdot_2 = (u - F) / I 
        # update_state
        qdot_1 += qdot_2 * ts
        q_1    += qdot_1 * ts
        q_3    += zdot * ts
    return qdot_1, qdot_2, zdot, q_1, q_3, f, v


#FUNCTION TO DEFINE THE PARAMETERS AS SYMBOLS
def symbolic_poly_2(poly_params, I, Fs, Fc, sigma_2, vs, ts, qdot_1, q_3, error, previous_error, error_sum, param_dict):
    
    g_v = Fc + (Fs - Fc) * math.exp(-(qdot_1/vs)**2) 
    zdot = qdot_1 - q_3 * abs(qdot_1) * poly_params[0] / g_v
    F = poly_params[0] * q_3 + poly_params[1] * zdot + sigma_2 * qdot_1
    F_eval = F.subs(param_dict)
    # update_state
    u = 12.0 * error + 0.03 * (error - previous_error) / ts + 0.0 * error_sum + F_eval
    qdot_2 = (u - F_eval) / I # acceleration
    return F, u, zdot, qdot_2, F_eval
  
def error_2(poly_params, params, xs, ys, vs, Fc, Fs, sigma_2, ts, force_rate, I):
    f = []
    for i in range(len(xs)):
        qdot_1 = 0.0      # x_dot
        qdot_2 = 0.0      # x_dot_dot
        q_3    = 0        # z
        q_1    = 0        # x
        u      = 0.0
        F      = 0.0
        F_last = 0.0
        u_ac = []
        F_ac = []
        error_sum = 0
        previous_error = 0
        error = xs[i] - qdot_1
        iteration = 0
        velocity = []
        time = []
        param_dict = dict(zip(poly_params, params))
        loop_iteration = 0
        
        for y in range(140):
            F, u, zdot, qdot_2, F_eval = symbolic_poly_2(poly_params, I, Fs, Fc, sigma_2, vs, ts, qdot_1, q_3, error, previous_error, error_sum, param_dict)
            F_ac.append(F_eval)
            qdot_1 += qdot_2.subs(param_dict) * ts # update velocity
            q_1    += qdot_1 * ts  # update position
            q_3    += zdot * ts    # 
            time.append(loop_iteration * ts)
            velocity.append(qdot_1)
            u_ac.append(u)
            error_sum += error * ts
            previous_error = error
            error = xs[i] - qdot_1
            
            F_last = poly_params[0] * q_3.subs(param_dict) + poly_params[1] * zdot.subs(param_dict) + sigma_2 * qdot_1
            loop_iteration += 1
            if q_1 < 0:
                q_1 += 360
            elif q_1 > 360:
                q_1 -= 360
                
            if abs(error) < 0.001:
                iteration += 1
            else:
                iteration = 0
            if iteration >= 15:
               # print("Error stable iteration: ", iteration)
                break 
        f.append((F_last-ys[i])**2)
#        print("reference Velocity: ", xs[i])
#        print("Loop iteration: ", loop_iteration)
#         show_model(velocity, velocity, time, 'time', 'velocity')
        
#         show_model(u_ac, u_ac, time, 'time', 'u')
#         show_model(F_ac, F_ac, time, 'time', 'Force')
#         show_model(F_ac, F_ac, velocity, 'Vel', 'Force')
    error_poli = sum(f)/len(f)
    return error_poli

#FUNCTION TO FIT THE PARAMETERS WITH THE ITERATIVE NEWTON METHOD

def newton_3(poly_params, xs, ys, param_value_guess, vs, Fc, Fs, s2, ts, force_rate, I, max_iter: int = 15, epsilon: float=0.001) -> Tuple[np.ndarray, int, float]:
    
    finalErrorsList = []
    params = np.array(param_value_guess)
    params_final = np.array(param_value_guess)
    final_error = 1e10    
    errorf = error_2(poly_params, params, xs, ys, vs, Fc, Fs, s2, ts, force_rate, I)
    X = sp.Matrix([errorf])
    Y = sp.Matrix(poly_params)
    myjac = X.jacobian(Y)
    xcurr = sp.Matrix(params)
    param_dict = dict(zip(poly_params, params))
    least_error = 1e10
    for iteration in range(max_iter):
        myjaceval = myjac.evalf(subs = param_dict)
        Xeval = X.evalf(subs=param_dict)
        xcurr = xcurr - myjaceval.pinv() @ Xeval
        for i, elem in enumerate(poly_params):
            param_dict[elem] = abs(xcurr[i])
        final_error = error_2(poly_params,params, xs, ys, vs, Fc, Fs, s2, ts, force_rate, I).subs(param_dict)
        if final_error <= least_error:
            least_error = final_error
            params_final = np.array(list(param_dict.values()),dtype=float)      
        if least_error <= epsilon:
            break
            
    return (params_final, least_error)

path            = 'data'                                      # use your folder name
file_name       = glob.glob("./" + path + "/*.csv")
points_quantity = len(file_name)
force           = []
velocity        = np.zeros(points_quantity)

for j in range(points_quantity):
    jnt_ctrl_torque     = []
    jnt_position        = []
    jnt_velocity        = []
    jnt_torque          = []
    jnt_command_current = []
    jnt_current         = []
    friction_torque     = []
    nominal_pos         = []
    nominal_vel         = []
    initial_position    = []
    initial_velocity    = []
    time                = []
    counter             = 0
    content             = genfromtxt(file_name[j], delimiter = ' ')
    velocity[j]         = float(file_name[j][16:22])

    for i in content[:, 0]:
        jnt_ctrl_torque.append(i)
        counter = counter + 1
        time.append(counter)
    for i in content[:, 1]:
        jnt_position.append(i)
    for i in content[:,2]:
        jnt_velocity.append(i)
    for i in content[:, 3]:
        jnt_torque.append(i)
    for i in content[:, 4]:
        jnt_command_current.append(i)
    for i in content[:, 5]:
        jnt_current.append(i)
    for i in content[:, 6]:
        friction_torque.append(i)
    for k in range(len(time)):
        time[k] = time[k] / 900
        
    x, y, yfit = fit_line(np.array(time), np.array(friction_torque))
    y_mean = np.mean(yfit)
    force.append(abs(y_mean))
    print("Point"+ str(j)+"=",abs(y_mean))

#SETUP THE VARIABLES 

Fs = get_breakaway_friction()
I = get_inertia()
print("Fs: ", Fs)
print("I: ", I)
poly_params = sp.symbols('Fc, vs, sigma_2')
init_param_values = np.array([1.5, 0.1, 0.1])

parameter, final_error = newton(poly_params, velocity, force, init_param_values, Fs)

F_fit = (parameter[0] + (Fs - parameter[0]) * np.exp(-(velocity/parameter[1])**2))*np.sign(velocity) + parameter[2]*velocity
#show_model(F_fit, force, velocity, 'Velocity', 'Force')
Fc      = parameter[0]
vs      = parameter[1]
sigma_2 = parameter[2]

print("Fc: ", Fc)
print("vs: ", vs)
print("Sigma 2: ", sigma_2)
print("FINAL ERROR: ", final_error)

ts = 0.01
poly_params_dyn = sp.symbols('s0, s1')
init_param_values = np.array([1, 1])
max_force = 1

parameter_dyn, final_error_dyn = newton_3(poly_params_dyn, velocity, force, init_param_values, vs, Fc, Fs, sigma_2, ts, max_force, I) 

print("Sigma 0: ", sigma_0)
print("Sigma 1: ", sigma_1)
print("FINAL ERROR: ", final_error_dyn)
