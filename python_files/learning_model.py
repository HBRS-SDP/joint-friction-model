# IMPORTANT NOTES

# 1 N = 1 (Kg * m)/s^2
# F = M * a                                       FORCE
# I = m * r^2  (Kg * m^2)                         INERTIA
# tau = I * a_ang                                 RELATION BETWEEN INERTIA AND ANGULAR ACCELERATION
# N * m = Kg * m^2 * rad / s^2 = (Kg * m^2)/s^2 

#USED LIBRARIES
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

def learn_model():
	    
#FUNCTION TO CALCULATE THE BREAKAWAY FRICTION FORCE FROM FILE

	def get_breakaway_friction():
		path = '../data/static_data/static_torque_breakaway_point.csv'
		rows = []
		data = pd.read_csv(path, sep = " ", header = None)
		Fc   = np.mean(data[0])
		return Fc

	#GET INERTIA FOR THE SPECIFIED JOINT

	def get_inertia():
		with open("../configs/constants.yml", "r") as file:
			documents = yaml.full_load(file)
			inertia   = documents["joint_inertia"][documents["test_joint"]]
		return inertia

	#FIT A LINE OVER ALL THE DATA TO GET A SINGLE VALUE OF FRICTION FORCE FOR EACH VELOCITY 
	# BASED ON LEAST SQUARE METHOD (REFERENCE): 
	#Golub, G. H., & Van Loan, C. F. (1980). An analysis of the total least squares problem. 
	#SIAM journal on numerical analysis, 17(6), 883-893.

	def get_average_friction(time, friction_torque):
		A        = np.hstack((time[np.newaxis].T, np.ones((len(time), 1))))
		b        = friction_torque[np.newaxis].T
		U, s, Vt = np.linalg.svd(A)
		s_inv    = np.array(s)

		for i in range(len(s_inv)):
			if abs(s_inv[i] - 0.) > 1e-5:
				s_inv[i] = 1. / s_inv[i]

		S_inv               = np.zeros((Vt.shape[0], U.shape[1]))
		np.fill_diagonal(S_inv, s_inv)
		A_pinv              = Vt.T.dot(S_inv).dot(U.T)
		X                   = A_pinv.dot(b)
		friction_torque_fit = X[0] * time + X[1]
		return time, friction_torque, friction_torque_fit

	#FUNCTION TO DEFINE THE PARAMETERS AS SYMBOLS OF FRICTION IN STEADY STATE (Fss)
	#EQUATION BASED ON: C. Canudas de Wit, H. Olsson, K. J. Astrom and P. Lischinsky, "A new model for control 
	#of systems with friction," in IEEE Transactions on Automatic Control, vol. 40, no. 3, pp. 419-425, March 1995

	def symbolic_polynomial_steady_state(poly_params, Fs, velocity):
		return ((poly_params[0] + (Fs - poly_params[0]) * sp.exp(-(velocity/poly_params[1])**2))*np.sign(velocity) + poly_params[2]*velocity)

	#FUNCTION TO CALCULATE THE ERROR OF THE MEASURED FRICCION AND THE GENERATED POLYNOMIAL
	#EQUATION BASED ON MEAN SQUARED ERROR IN: Sammut, C., & Webb, G. I. (Eds.). (2010). 
	#Encyclopedia of Machine Learning. Springer US. https://doi.org/10.1007/978-0-387-30164-8.

	def error_polynomial_steady_state(poly_params, velocity, friction_force, Fs):
		cumulative_error = []
		for i in range(len(velocity)):
			cumulative_error.append((symbolic_polynomial_steady_state(poly_params, Fs, velocity[i])-friction_force[i])**2)
		return sum(cumulative_error)/len(cumulative_error)

	#FUNCTION TO FIT THE PARAMETERS WITH THE ITERATIVE NEWTON-RAPHSON METHOD
	#REFERENCE: Ben-Israel, A. (1966). A Newton-Raphson method for the solution of systems of equations. 
	#In Journal of Mathematical Analysis and Applications (Vol. 15, Issue 2, pp. 243–252). Elsevier BV.

	def newton_raphson_steady_state(poly_params, velocity, friction_force, param_value_guess, Fs, max_iter: int=100, epsilon: float=0.01):
		params       = np.array(param_value_guess)
		params_final = np.array(param_value_guess)
		final_error  = 1e10    
		errorf       = error_polynomial_steady_state(poly_params, velocity, friction_force, Fs)
		X            = sp.Matrix([errorf])
		Y            = sp.Matrix(poly_params)
		myjac        = X.jacobian(Y)
		xcurr        = sp.Matrix(params)
		param_dict   = dict(zip(poly_params, params))
		least_error  = 1e10
		for iteration in range(max_iter):
			myjaceval = myjac.evalf(subs = param_dict)
			Xeval     = X.evalf(subs = param_dict)
			xcurr     = xcurr - myjaceval.pinv() @ Xeval
			for i,elem in enumerate(poly_params):
				param_dict[elem] = abs(xcurr[i])
			final_error = error_polynomial_steady_state(poly_params, velocity, friction_force, Fs).subs(param_dict)
			if final_error <= least_error:
				least_error = final_error
				params_final = np.array(list(param_dict.values()), dtype=float)
			if final_error <= epsilon:
				break
		return (params_final, least_error)

	#FUNCTION TO PLOT 2 FUNCTIONS IN A SINGLE PLOT

	def show_model(new_data, data, reference, xlabel_name, ylabel_name, title):
		plt.figure(figsize = (10, 10))
		plt.plot(reference, new_data, label='Estimated data')
		plt.grid(color = 'b', linestyle = '-', linewidth = 0.1)
		plt.scatter(reference, data, color = 'r', linewidth = 1, label='Original data')
		plt.legend()
		plt.xlabel(xlabel_name)
		plt.ylabel(ylabel_name)
		plt.title(title)
		plt.savefig('../plots/' + title + '.png')

	#SIMULATION WITH MASS (PLOTTING PURPOSES ONLY)

	def sim_mass_with_ramp_force_input(t, I, Fs, Fc, sigma_0, sigma_1, sigma_2, vs, force_rate, ts):
		zdot                 = 0.0                     # z_dot
		acceleration         = 0.0                     #x_dot_dot
		velocity             = 0.0                     #x_dot
		z                    = 0.0                     # z
		angular_position     = 0.0                     # x
		velocity_accumulated = []
		friction_accumulated = []
		
		for i in t:
			zdot    = velocity - ((z * abs(velocity) * sigma_0) / (Fc + (Fs - Fc) * sp.exp(-(velocity/vs)**2)))
			F       = sigma_0 * z + sigma_1 * zdot + sigma_2 * velocity
			u       = force_rate * i    
			acceleration  = (u - F) / I 
			friction_accumulated.append(F)
			velocity_accumulated.append(velocity)
			
			# update_state
			velocity         += acceleration * ts
			angular_position += velocity * ts
			z                += zdot * ts
		return friction_accumulated, velocity_accumulated


	#FUNCTION TO DEFINE THE PARAMETERS OF THE LUGRE FUNCTION AS SYMBOLS

	def symbolic_polynomial_dynamic(poly_params, I, Fs, Fc, sigma_2, vs, ts, velocity, z, error, previous_error, error_sum, param_dict):
		
		g_v          = Fc + (Fs - Fc) * sp.exp(-(velocity / vs)**2) 
		zdot         = velocity - z * abs(velocity) * poly_params[0] / g_v
		F            = poly_params[0] * z + poly_params[1] * zdot + sigma_2 * velocity
		F_eval       = F.subs(param_dict)
		
		# update_state
		u            = 12.0 * error + 0.03 * (error - previous_error) / ts + 0.0 * error_sum + F_eval
		acceleration = (u - F_eval) / I                                                # acceleration
		return F, u, zdot, acceleration, F_eval

	#FUNCTION TO DEFINE THE ERROR OF THE LUGRE FUNCTION AS SYMBOLS

	def error_polynomial_lugre(poly_params, params, velocity, friction_force, vs, Fc, Fs, sigma_2, ts, force_rate, I):
		f = []
		for i in range(len(velocity)):
			velocity_current     = 0.0        # x_dot
			acceleration         = 0.0        # x_dot_dot
			z                    = 0.0        # z
			angular_position     = 0.0        # x
			u                    = 0.0
			F                    = 0.0
			F_last               = 0.0
			u_accumulated        = []
			F_accumulated        = []
			error_sum            = 0
			previous_error       = 0
			error                = velocity[i] - velocity_current
			iteration            = 0
			velocity_accumulated = []
			time                 = []
			param_dict           = dict(zip(poly_params, params))
			loop_iteration       = 0
			
			for y in range(140):
				F, u, zdot, acceleration, F_eval = symbolic_polynomial_dynamic(poly_params, I, Fs, Fc, sigma_2, vs, ts, velocity_current, z, error, previous_error, error_sum, param_dict)
				F_accumulated.append(F_eval)
				velocity_current += acceleration.subs(param_dict) * ts # update velocity
				angular_position += velocity_current * ts  # update position
				z                += zdot * ts    # 
				time.append(loop_iteration * ts)
				velocity_accumulated.append(velocity_current)
				u_accumulated.append(u)
				error_sum        += error * ts
				previous_error    = error
				error             = velocity[i] - velocity_current
				
				F_last = poly_params[0] * z.subs(param_dict) + poly_params[1] * zdot.subs(param_dict) + sigma_2 * velocity_current
				loop_iteration   += 1
				if angular_position < 0:
					angular_position += 2 * np.pi
				elif angular_position > 2 * np.pi:
					angular_position -= 2 * np.pi
					
				if abs(error) < 0.001:
					iteration += 1
				else:
					iteration = 0
				if iteration >= 15:
					break 
			f.append((F_last-friction_force[i])**2)
	#        print("reference Velocity: ", velocity[i])
	#        show_model(velocity, velocity, time, 'Time [sec]', 'Velocity [rad/sec]', 'Control of velocity')
			
		error_poli = sum(f)/len(f)
		return error_poli

	#FUNCTION TO FIT THE PARAMETERS WITH THE ITERATIVE NEWTON METHOD

	def newton_raphson_lugre(poly_params, velocity, friction_force, param_value_guess, vs, Fc, Fs, s2, ts, force_rate, I, max_iter: int = 15, epsilon: float=0.001):
		
		finalErrorsList = []
		params = np.array(param_value_guess)
		params_final = np.array(param_value_guess)
		final_error = 1e10    
		errorf = error_polynomial_lugre(poly_params, params, velocity, friction_force, vs, Fc, Fs, s2, ts, force_rate, I)
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
			final_error = error_polynomial_lugre(poly_params,params, velocity, friction_force, vs, Fc, Fs, s2, ts, force_rate, I).subs(param_dict)
			if final_error <= least_error:
				least_error = final_error
				params_final = np.array(list(param_dict.values()),dtype=float)      
			if least_error <= epsilon:
				break
				
		return (params_final, least_error)

	#GET DATA FROM FILES SAVED FROM KINOVA ARM

	path            = 'data'                                      # using the folder name
	file_name       = glob.glob("../" + path + "/*.csv")
	points_quantity = len(file_name)
	force           = []
	velocity        = np.zeros(points_quantity)

	with open("../configs/constants.yml", 'r') as config_file:
		constants = yaml.full_load(config_file)
		frequency = constants["RATE_HZ"]

	for j in range(points_quantity):
		jnt_ctrl_torque     = []
		jnt_position        = []
		jnt_velocity        = []
		jnt_torque          = []
		jnt_command_current = []
		jnt_current         = []
		friction_torque     = []
		time                = []
		counter             = 0
		content             = genfromtxt(file_name[j], delimiter = ' ')
		velocity[j]         = float(file_name[j][17:24])

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
			if(math.isinf(i)):
				friction_torque.append(0)
			else:
				friction_torque.append(i)
		for k in range(len(time)):
			time[k] = time[k] / frequency
					
		time, friction_torque, friction_torque_fit = get_average_friction(np.array(time), np.array(friction_torque))
		friction_mean = np.mean(friction_torque_fit)
		force.append(abs(friction_mean))
		
	#SETUP THE VARIABLES 

	Fs = get_breakaway_friction()
	I  = get_inertia()
	print("Fs: ", "{:.2f}".format(Fs), "[N]")
	print("I: ", "{:.2f}".format(I), "[Kg * m^2]")
	poly_params = sp.symbols('Fc, vs, sigma_2')
	init_param_values = np.array([1.5, 0.1, 0.1])

	parameter, final_error = newton_raphson_steady_state(poly_params, velocity, force, init_param_values, Fs)

	Fc      = parameter[0]
	vs      = parameter[1]
	sigma_2 = parameter[2]

	print("Fc: ","{:.2f}".format(Fc), "[N]")
	print("vs: ", "{:.2f}".format(vs),"[rad/sec]" )
	print("Sigma 2: ", "{:.2f}".format(sigma_2))
	print("Error of friction with estimated parameters in steady state: ", "{:.4f}".format(final_error))

	ts = 0.01
	poly_params_dyn   = sp.symbols('s0, s1')
	init_param_values = np.array([1, 1])
	max_force         = 1

	parameter_dyn, final_error_dyn = newton_raphson_lugre(poly_params_dyn, velocity, force, init_param_values, vs, Fc, Fs, sigma_2, ts, max_force, I) 
	sigma_0 = parameter_dyn[0]
	sigma_1 = parameter_dyn[1]

	print("Sigma 0: ", "{:.2f}".format(sigma_0))
	print("Sigma 1: ", "{:.2f}".format(sigma_1))
	print("Error of Lugre model with estimated parameters: ", "{:.4f}".format(final_error_dyn))
	
	#PLOT THE COMPARISON BETWEEN ORIGINAL DATA AND LUGRE MODEL ESTIMATED

	time_span =  np.linspace(0, 1.5, 151)
	F_rate = 1
	v = np.linspace(0, 1.5, 151)
	friction_force_estimated, velocity_estimated = sim_mass_with_ramp_force_input(time_span, I, Fs, Fc, sigma_0, sigma_1, sigma_2, vs, F_rate, time_span[1] - time_span[0])
	force_real = np.zeros(len(v))
	force_real[0] = Fs
	counter = 0
	velocity = list(velocity)
	friction_force_estimated_steady_state = []

	for i in v:
		if round(i, 2) in velocity:
			index = velocity.index(round(i, 2))
			force_real[counter] = force[index]
		counter += 1
		friction_force_estimated_steady_state.append((Fc + (Fs - Fc) * sp.exp(-(i/vs)**2)) * np.sign(i) + sigma_2 * i)
		
	show_model(friction_force_estimated_steady_state, force_real, v, 'Velocity [rad/sec]', 'Friction force [N]', 'Friction force in steady state')
	show_model(friction_force_estimated + Fc, force_real, v, 'Velocity [rad/sec]', 'Friction force [N]', 'Friction force given by LuGre')