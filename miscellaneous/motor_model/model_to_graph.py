#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import yaml
import math
import sys
import os

# use_linear_term = False

# if use_linear_term:
#     print('\033[1m' + 'The linear term is not yet supported by the MRS UAV System')
#     print('!!! THE PRODUCED THRUST CURVE PARAMETERS WILL NOT WORK !!!')
#     print('\033[0m')

# g = 9.81

# if(len(sys.argv) != 2):
#     print("Please provide exactly 1 argument - path to yaml file with measurement data")
#     quit()
# filename=sys.argv[1]

# with open(filename, 'r') as file:
#     measured_values = yaml.safe_load(file)

# num_motors = 0
# mass = []
# throttle = []

# if 'mass' not in measured_values:
#     print("mass not found in the input file!\nQuitting...")
#     quit()
# if 'num_motors' not in measured_values:
#     print("num_motors not found in the input file!\nQuitting...")
#     quit()

# num_motors = measured_values['num_motors']
# mass = measured_values['mass']

# if 'throttle' in measured_values:
#     throttle = measured_values['throttle']

# elif ('pwm_min' in measured_values) and ('pwm_max' in measured_values) and ('pwm_throttle' in measured_values):
#     pwm_min = measured_values['pwm_min']
#     pwm_max = measured_values['pwm_max']
#     pwm_throttle = measured_values['pwm_throttle']
#     for i in pwm_throttle:
#         throttle.append((i - pwm_min) / (pwm_max - pwm_min))

# # adding 0 throttle = 0 thrust improves the fit when linear term is used.
# if use_linear_term:
#     mass.append(0.0)
#     throttle.append(0.0)

# if(len(mass) != len(throttle)):
#     print("mass and throttle lists have different lengths!\nQuitting...")
#     quit()

# mass_per_motor = [0]*(len(mass))
# for i in range(len(mass)):
#     mass_per_motor[i] = mass[i] / num_motors # divide mass by number of motors to get mass per 1 motor

# A=np.ones((len(mass),3))

# for idx, x in enumerate(mass_per_motor):
#     A[idx, 0] = math.sqrt((x*g));

#     if use_linear_term:
#         A[idx, 1] = x*g;
#     else:
#         A[idx, 1] = 0.0;

#     A[idx, 2] = 1.0;

# B=np.array(throttle)

# C=np.linalg.lstsq(A,B,rcond=None)

# # print(C[0])
# ka=C[0][0] # quadratic term
# kb=C[0][1] # linar term
# kc=C[0][2] # constant term

# # round the parameters and convert them to strings (to avoid scientific notation in the yaml file)
# rounded_ka=f"{ka:.6f}"
# rounded_kb=f"{kb:.6f}"
# rounded_kc=f"{kc:.6f}"

# print("quadratic term: " + rounded_ka)
# if use_linear_term:
#     print("linear term: " + rounded_kb)
# print("constant term: " + rounded_kc + "\n")

# thrust_50 = 0
# thrust_100 = 0
# if use_linear_term:
#     thrust_50 = (-ka*g*math.sqrt((ka**2)-4*kb*kc + 4*kb*0.5)+(ka**2)*g-2*kb*kc*g+2*kb*0.5*g)/(2*(kb**2)*(g**2))
#     thrust_100 = (-ka*g*math.sqrt((ka**2)-4*kb*kc + 4*kb*1.0)+(ka**2)*g-2*kb*kc*g+2*kb*1.0*g)/(2*(kb**2)*(g**2))
# else:
#     thrust_50 = (1/g)*((0.5-kc)/ka)**2
#     thrust_100 = (1/g)*((1.0-kc)/ka)**2

# print('\033[1m' + 'Sanity check: do these values make sense?')
# print('\033[0m')
# print(f'Thrust of a single motor at 50% throttle: {thrust_50:.2f} kg-force')
# print(f'Thrust of a single motor at 100% throttle: {thrust_100:.2f} kg-force\n')

# print_motors = num_motors
# if num_motors == 1:
#     print_motors = 4
# print(f'Thrust of a drone with {print_motors} motors at 50% throttle: {(print_motors*thrust_50):.2f} kg-force')
# print(f'Thrust of a drone with {print_motors} motors at 100% throttle: {(print_motors*thrust_100):.2f} kg-force')


# if use_linear_term:
#     data = {
#         'thrust_params': {
#             'n_motors': int(print_motors),\
#             'k_quad': rounded_ka,\
#             'k_lin': rounded_kb,\
#             'k_const': rounded_kc}
#     }

#     with open('tmp.yaml', 'w') as outfile:
#         outfile.write('# THIS VERSION OF THRUST CURVE IS NOT YET IMPLEMENTED IN THE MRS UAV SYSTEM\n')
#         outfile.write('# DO NOT USE\n\n')
#         outfile.write('# Thrust curve parameters\n')
#         outfile.write('# k_quad - quadratic term\n')
#         outfile.write('# k_lin - linear term\n')
#         outfile.write('# k_const - constant term\n')
#         outfile.write('# n_motors - number of motors on the UAV\n\n')
#         yaml.dump(data, outfile, default_style=None, default_flow_style=False)
# else:
#     data = {
#         'motor_params': {
#             'n_motors': int(print_motors),\
#             'a': rounded_ka,\
#             'b': rounded_kc}
#     }

#     with open('tmp.yaml', 'w') as outfile:
#         outfile.write('# Motor thrust curve parameters\n')
#         outfile.write('# a - quadratic term\n')
#         outfile.write('# b - constant term\n')
#         outfile.write('# n_motors - number of motors on the UAV\n\n')
#         yaml.dump(data, outfile, default_flow_style=False)

# # The yaml file produced by the yaml.dump contains quotes, as the rounded floats are written as strings (to avoid scientific notation)
# # The following code will simply remove the quotes

# with open(r'tmp.yaml', 'r') as infile, open(r'output.yaml', 'w') as outfile:
#     data = infile.read()
#     data = data.replace("'", "")
#     outfile.write(data)

# os.remove("tmp.yaml")
# print('\nOutput written to output.yaml')

# if use_linear_term:
#     print('\033[1m' + '\nThe linear term is not yet supported by the MRS UAV System')
#     print('!!! THE PRODUCED THRUST CURVE PARAMETERS WILL NOT WORK !!!')
#     print('\033[0m')

# Following section is only for plotting

Thr = np.arange(0, 1, 0.01)
Ms = [0]*(len(Thr))
ka = 0.5445
kc = -0.4664
g = 9.81
for idx, x in enumerate(Thr):
    # if use_linear_term:
    #     term=ka**2-4*kb*kc + 4*kb*x
    #     if term <= 0:
    #         Ms[idx] = 0
    #     else:
    #         Ms[idx] = (-ka*g*math.sqrt((ka**2)-4*kb*kc + 4*kb*x)+(ka**2)*g-2*kb*kc*g+2*kb*x*g)/(2*(kb**2)*(g**2))
    # else:
      Ms[idx] = (1/g)*((x-kc)/ka)**2

# plt.scatter(throttle, mass_per_motor, label="data points", color="red")
plt.plot(Thr, Ms, label="fit curve")
plt.legend()
plt.title("Thrust fit for 1 motor")
plt.xlabel("Throttle")
plt.ylabel("Thrust (kg-force)")
plt.show()
