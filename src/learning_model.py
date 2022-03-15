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

def learn_model():
    path = 'data' # use your folder name
    file_name = glob.glob("../" + path + "/*.csv")
    velocity = np.array([0.1,0.3,0.5,0.9,1.3])

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
        return (y_fit)
    
    force=[]
    for j in range(len(file_name)):
        friction_torque=[]
        time=[]
        counter=0
        content = genfromtxt(file_name[j], delimiter=' ')
        for i in content[:,6]:
            friction_torque.append(i)
            counter=counter+1
            time.append(counter)
        for k in range(len(time)):
            time[k]=time[k]/900
        
        yfit = fit_line(np.array(time), np.array(friction_torque))
        y_mean = np.mean(yfit)
        force.append(abs(y_mean))
        print("Point"+ str(j)+"=",abs(y_mean))

    #FUNCTION TO DEFINE THE PARAMETERS AS SYMBOLS
    def symbolic_poly(poly_params, Fs, v):
        return ((poly_params[0] + (Fs - poly_params[0]) * sp.exp(-(v/poly_params[1])**2))*np.sign(v) + poly_params[2]*v)
    #FUNCTION TO CALCULATE THE ERROR OF THE ORIGINAL FUNCTION AND THE FUNCTION WITH NEW PARAMETERS
    def error(poly_params, xs, ys, Fs):
        aux = ([])
        for i in range(len(xs)):
            aux.append(symbolic_poly(poly_params, Fs, xs[i])-ys[i])
        return (reduce(lambda y, x: x**2 + y, aux))/(len(aux))

    #FUNCTION TO FIT THE PARAMETERS WITH THE ITERATIVE NEWTON METHOD
    def newton(poly_params, xs: np.ndarray, ys: np.ndarray, param_value_guess: np.ndarray,
        Fs, max_iter: int=100, epsilon: float=0.01) -> Tuple[np.ndarray, int, float]:

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
                param_dict[elem] = xcurr[i]
            final_error = error(poly_params, xs, ys, Fs).subs(param_dict)
            #print(final_error, np.array(list(param_dict.values()),dtype=float))
            if final_error <= least_error:
                least_error = final_error
                params_final = np.array(list(param_dict.values()),dtype=float)
            if final_error <= epsilon:
                break
        #params = np.array(list(param_dict.values()),dtype=float)
        return (params_final, least_error)

    #SETUP THE VARIABLES 
    poly_params = sp.symbols('Fc, vs, beta')
    init_param_values = np.array([1, 0.5, 2])
    Fs= 2
    parameter, final_error = newton(poly_params, velocity, force, init_param_values, Fs)

    #FUNCTION TO PLOT MODELS
    def show_model(y_fit, y, x, xlabel, ylabel):
    # plt.figure(figsize=(3,3))
        plt.plot(x,y_fit)
        plt.grid(color='b', linestyle='-', linewidth=0.1)
        plt.scatter(x,y, color='r', linewidth=1)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.show()

    F_fit = (parameter[0] + (Fs - parameter[0]) * np.exp(-(velocity/parameter[1])**2))*np.sign(velocity) + parameter[2]*velocity
    show_model(F_fit, force, velocity, 'Velocity', 'Force')
    print("Fc: ", parameter[0])
    print("vs: ", parameter[1])
    print("Sigma 2: ", parameter[2])
    print("FINAL ERROR: ", final_error)

    vs = parameter[1]
    Fc = parameter[0]
    sigma_2 = parameter[2]

    #FUNCTION TO DEFINE THE PARAMETERS AS SYMBOLS
    def symbolic_poly_2(poly_params, M, Fs, Fc, sigma_2, vs, ts, force_rate, u, qdot_1, qdot_3, q_1, q_3):
        exponential = reduce(lambda x, y: math.exp(-(x/vs)**2) + y, qdot_1)
        g_v = Fc + (Fs - Fc) * exponential 
        absolute_value = reduce(lambda x, y: abs(x) + y, qdot_1)
        qdot_3 = qdot_1[0] - q_3 * absolute_value * poly_params[0] / g_v
        F = poly_params[0] * q_3 + poly_params[1] * qdot_3 + sigma_2 * qdot_1[0]
        qdot_2 = (u - F) / M
        
        # update_state
        qdot_1[0] += qdot_2 * ts
        q_1    += qdot_1[0] * ts
        q_3    += qdot_3 * ts
        u      += force_rate * ts

        return F, qdot_1[0], q_1, q_3, u, qdot_3
    
    def error_2(poly_params, params, xs, ys, vs, Fc, Fs, sigma_2, ts, force_rate):
        f = []
        for i in range(len(xs)):
            qdot_3 = 0     # z_dot
            qdot_1 = [0]   # x_dot
            q_3 = 0        # z
            q_1 = 0        # x
            u = 0.0
            M = 1
            F = 0
            sim_velocity = 0.0
            while sim_velocity < xs[i]:
                F, qdot_1[0], q_1, q_3, u, qdot_3 = symbolic_poly_2(poly_params, M, Fs, Fc, sigma_2, vs, ts, force_rate, u, qdot_1, qdot_3, q_1, q_3)
                param_dict = dict(zip(poly_params, params))
                sim_velocity = qdot_1[0].subs(param_dict)
            f.append((F-ys[i])**2)
        return sum(f)/len(f)

    #FUNCTION TO FIT THE PARAMETERS WITH THE ITERATIVE NEWTON METHOD
    def newton_3(poly_params, xs, ys, param_value_guess, vs, Fc, Fs, s2, ts, force_rate, max_iter: int=100, epsilon: float=0.001) -> Tuple[np.ndarray, int, float]:
        
        finalErrorsList = []
        params = np.array(param_value_guess)
        params_final = np.array(param_value_guess)
        final_error = 1e10    
        errorf = error_2(poly_params, params, xs, ys, vs, Fc, Fs, s2, ts, force_rate)
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
                param_dict[elem] = xcurr[i]    
            final_error = error_2(poly_params,params, xs, ys, vs, Fc, Fs, s2, ts, force_rate).subs(param_dict).subs(param_dict)
            if final_error <= least_error:
                least_error = final_error
                params_final = np.array(list(param_dict.values()),dtype=float)
                
                #ploting the error for every iteration
                #finalErrorsList.append(final_error)
                #l = list(range(len(finalErrorsList)))
                #plot_f(l, finalErrorsList, 'Error over time', '#iteration', 'error')
                
            if least_error <= epsilon:
                print("Num interations ", iteration)
                break
                
        return (params_final, least_error)

    ts = 0.3
    time_span = 1.5
    t_sol = np.arange(0, time_span, ts)

    poly_params_dyn = sp.symbols('s0, s1')
    init_param_values = np.array([10, 10])
    max_force = 1
    parameter_dyn, final_error_dyn = newton_3(poly_params_dyn, velocity, force, init_param_values, vs, Fc, Fs, sigma_2, ts, max_force) 
    print("Sigma 0: ", parameter_dyn[0])
    print("Sigma 1: ", parameter_dyn[1])
    print("FINAL ERROR: ", final_error_dyn)

    sigma_0= parameter_dyn[0]
    sigma_1= parameter_dyn[1]

    def sim_mass_with_ramp_force_input(t, M, Fs, Fc, sigma_0, sigma_1, sigma_2, vs, force_rate, ts):
        qdot_3 = 0.0 # z_dot
        qdot_2 = 0.0 #x_dot_dot
        qdot_1 = 0.0 #x_dot , q_2
        q_3 = 0.0 # z
        q_1 = 0.0 # x
        v = []
        f = []
        
        for i in t:
            zdot = qdot_1 - ((q_3 * abs(qdot_1) * sigma_0) / (Fc + (Fs - Fc) * np.exp(-(qdot_1/vs)**2)))
            F = sigma_0 *q_3 + sigma_1 * qdot_3 + sigma_2 * qdot_1
            f.append(F)
            v.append(qdot_1)
            u = force_rate * i     # % ramped-up force input
            qdot_2 = (u - F) / M
            qdot_3 = zdot
            # update_state
            qdot_1 = qdot_1 + qdot_2 * ts
            q_1 = q_1 + qdot_1 * ts
            q_3 = q_3 + qdot_3 * ts

        return qdot_1, qdot_2, qdot_3, q_1, q_3, f, v

    M = 1
    Fc_approx = parameter[0]
    vs_approx = parameter[1]
    sigma_2_approx = parameter[2]
    sigma_1_approx = parameter_dyn[1]
    sigma_0_approx = parameter_dyn[0]

    time_span =  np.linspace(0, 2, 100)
    F_rate =1
    # qdot_1, qdot_2, qdot_3, q_1, q_3, friction_force, velocity = sim_mass_with_ramp_force_input(time_span, M, Fs, Fc, sigma_0, sigma_1, sigma_2, vs, F_rate, time_span[1] - time_span[0])
    qdot_1, qdot_2, qdot_3, q_1, q_3, friction_force_estimated, velocity_estimated = sim_mass_with_ramp_force_input(time_span, M, Fs, Fc_approx, sigma_0_approx, sigma_1_approx, sigma_2_approx, vs_approx, F_rate, time_span[1] - time_span[0])

    # show_model(velocity_estimated, velocity, friction_force, 'Velocity', 'Force')
    show_model(friction_force_estimated, friction_force_estimated, time_span, 'time', 'Force')
