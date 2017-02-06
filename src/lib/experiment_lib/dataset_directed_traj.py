# -*- coding: utf-8 -*-
"""
@author: maestre
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
lib_path = os.path.abspath(os.path.join('..', 'a2l_core_lib'))
sys.path.append(lib_path)
import ros_services
import simulation_parameters as sim_param
from numpy import linspace
import copy

def plot_setup():
    
    # plot figure
    fig = plt.figure()
    fig.clf()
    ax = Axes3D(fig)

    obj_pos = [0.65, 0, -0.14]
    obj_side = 0.1
    
    # box init_pos
    ax.bar3d(obj_pos[0] - obj_side/2, 
             obj_pos[1] - obj_side/2, 
             obj_pos[2] - obj_side/2, 
             [.1], [.1], [.1], 
             color='lightgrey',
             alpha=0.2,
             edgecolor='none')

    # robot
    robot_width = .2
    robot_height = .6
    robot_length = .4
    ax.bar3d(-robot_width/2, 
             -robot_length/2, 
             -robot_height/2, 
             robot_width, robot_length, robot_height, 
             color='lightgrey',
             alpha=0.2,
             edgecolor='none')

    # limits
    lim = .3
    ax.set_xlim3d([obj_pos[0]-lim, obj_pos[0]+lim])
    ax.set_ylim3d([obj_pos[1]-lim, obj_pos[1]+lim])
    ax.set_zlim3d([obj_pos[2]-lim, obj_pos[2]+lim])
    
    # labels
    plt.xlabel('xlabel')
    plt.ylabel('ylabel')

    return ax

def plot_traj(ax, traj_vector):
    
    for traj in traj_vector:    
        ## plot traj from init pos
        eef_pos_vector_x = [pos[0] for pos in traj]
        eef_pos_vector_y = [pos[1] for pos in traj]
        eef_pos_vector_z = [pos[2] for pos in traj]
        ax.plot(eef_pos_vector_x, 
                eef_pos_vector_y, 
                eef_pos_vector_z,
                '-')#,
                #color = 'blue')
        ax.plot(eef_pos_vector_x, 
                eef_pos_vector_y, 
                eef_pos_vector_z,
                '*',                                     
                markersize=3,
                c='grey')
            
def plot_traj_eef_obj(ax, pos_rot_vector):
    
    
    eef_traj_x = []
    eef_traj_y = []
    eef_traj_z = []
    for pos in range(0, len(pos_rot_vector), 12):
        current_x = float(pos_rot_vector[pos+0])
        eef_traj_x.append(current_x)
        current_y = float(pos_rot_vector[pos+1])
        eef_traj_y.append(current_y)
        current_z = float(pos_rot_vector[pos+2])
        eef_traj_z.append(current_z)
    
    ax.plot(eef_traj_x, 
            eef_traj_y, 
            eef_traj_z,
            '-')            
    ax.plot(eef_traj_x, 
            eef_traj_y, 
            eef_traj_z,
            '*',                                     
            markersize=3,
            c='grey')                 

def create_diverse_trajs(traj,
                         obj_pos,
                         effect):
#    traj_vector = []    
#    for nb_traj in range(nb_diverse_trajs):   
        tmp_traj = [] ## [eef_pos eef_orien obj_pos obj_orien] = [float]
        
        ## initial pos
#        tmp_traj.append(traj[0][0])
#        tmp_traj.append(traj[0][1])
#        tmp_traj.append(traj[0][2])
        zero_vector = [0,0,0]        
#        tmp_traj += zero_vector        
#        tmp_traj += obj_pos
#        tmp_traj += zero_vector         
        
#        for traj_wp in traj[1:-1]:            
        for traj_wp in traj:
            orig_traj_x = round(traj_wp[0], sim_param.round_value+2)
            orig_traj_y = round(traj_wp[1], sim_param.round_value+2)
            orig_traj_z = round(traj_wp[2], sim_param.round_value+2)
                        
            ## eef pos
#            tmp_eef_pos = []
#            tmp_eef_pos.append(round(orig_traj_x + random.uniform(-traj_change, traj_change), 
#                                     round_value+2))
#            tmp_eef_pos.append(round(orig_traj_y + random.uniform(-traj_change, traj_change), 
#                                     round_value+2))
#            tmp_eef_pos.append(round(orig_traj_z + random.uniform(-traj_change, traj_change), 
#                                     round_value+2))
#            tmp_traj += tmp_eef_pos
            tmp_traj += [orig_traj_x, orig_traj_y, orig_traj_z]
            
            ## eef_orien                         
            tmp_traj += zero_vector
            
            ## obj pos and orientation
            tmp_traj += obj_pos
            tmp_traj += zero_vector
            
        ## final obj pos
        tmp_traj.append(traj[-1][0])
        tmp_traj.append(traj[-1][1])
        tmp_traj.append(traj[-1][2])
        
        tmp_traj += zero_vector
        displ_vector = [0,0,0]
        if effect == 'right':
            displ_vector[1] -= sim_param.obj_displacement
        elif effect == 'left':
            displ_vector[1] += sim_param.obj_displacement
        elif effect == 'close':
            displ_vector[0] -= sim_param.obj_displacement            
        elif effect == 'far':
            displ_vector[0] += sim_param.obj_displacement
        tmp_obj_pos = [x+y for x,y in zip(obj_pos, displ_vector)]
        tmp_traj += tmp_obj_pos
        tmp_traj += zero_vector                     
            
#        print(nb_traj, tmp_traj)
#        traj_vector.append(tmp_traj)
#    return traj_vector
        return [tmp_traj]
    
def generate_dataset(effect):
    
    ''' Get obj end eef states '''
    eef_pos = ros_services.call_get_eef_pose('left')
    eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos[0:3]]
    print("eef_pos", eef_pos)
    orig_eef_pos = copy.copy(eef_pos)
    
    obj_pos = ros_services.call_get_model_state(sim_param.obj_name)
    obj_pos = [round(pos, sim_param.round_value) for pos in obj_pos[0:3]]
    print('obj_pos', obj_pos)
    
    ## all trajs converge into a mid pos
    mid_pos = copy.copy(obj_pos)
    mid_pos[1] += 0.1
    
    traj_vector = []
    traj_diverse_vector = []
    for i in range(nb_init_traj):
        eef_pos = copy.copy(orig_eef_pos)
        ''' Create new inital position '''
        if i != 0:
            eef_pos_range = sim_param.new_obj_pos_dist
            eef_pos[0] = eef_pos[0] + random.uniform(-eef_pos_range,eef_pos_range*2)
#            eef_pos[0] = eef_pos[0] - random.uniform(,eef_pos_range)                                              
    
            eef_pos[1] = eef_pos[1] + random.uniform(-eef_pos_range*2,eef_pos_range*2)
#            eef_pos[1] = eef_pos[1] - random.uniform(-eef_pos_range,eef_pos_range)
            
            eef_pos[2] = eef_pos[2] + random.uniform(-eef_pos_range,eef_pos_range)
#            eef_pos[2] = eef_pos[2] - random.uniform(-eef_pos_range,eef_pos_range)
            eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos]        
    
        ''' Create main traj '''
#        var_x_vector = (linspace(eef_pos[0], obj_pos[0], nb_steps-3)).tolist()
##        var_x_vector = linspace(eef_pos[0], obj_pos[0], nb_steps)
#        var_x_vector = [round(pos, sim_param.round_value+2) for pos in var_x_vector]
#        var_x_vector = var_x_vector + [var_x_vector[-1]] + [var_x_vector[-1]] + [var_x_vector[-1]]
#        
#        var_y_vector = linspace(eef_pos[1], obj_pos[1], nb_steps)
#        var_y_vector = [round(pos, sim_param.round_value+2) for pos in var_y_vector]
#    #    var_y_vector = [var_y_vector[0]] + var_y_vector    
#        
##        var_z = abs(eef_pos[2] - obj_pos[2])
#        var_z_vector = (linspace(eef_pos[2], obj_pos[2], int(nb_steps/2))).tolist()
#        var_z_vector = [round(pos, sim_param.round_value+2) for pos in var_z_vector]
##        var_z_vector = var_z_vector + [var_z_vector[-1]] + [var_z_vector[-1]] + [var_z_vector[-1]] + [var_z_vector[-1]]
#        var_z_vector_tmp = [obj_pos[2] for i in range(len(var_z_vector),nb_steps)]
#        var_z_vector += var_z_vector_tmp

        mid_var = traj_change * 5
        if eef_pos[0] < obj_pos[0]:
            mid_vax_x = mid_pos[0] + random.uniform(-mid_var,0)
        else:
            mid_vax_x = mid_pos[0] + random.uniform(0,mid_var)
        mid_vax_y = mid_pos[1] + random.uniform(-mid_var,mid_var)
        mid_vax_z = mid_pos[2] + random.uniform(0,mid_var*2)
        
        var_x_vector = (linspace(eef_pos[0], mid_vax_x,
                        int(nb_steps/2))).tolist()
        var_x_vector = [round(pos, sim_param.round_value+2) for pos in var_x_vector]
        var_x_vector_tmp = [mid_vax_x] + [mid_pos[0] for i in range(len(var_x_vector),nb_steps)][1:]
        var_x_vector += var_x_vector_tmp
        
        var_y_vector = (linspace(eef_pos[1], mid_vax_y,
                        int(nb_steps/2))).tolist()
        var_y_vector = [round(pos, sim_param.round_value+2) for pos in var_y_vector]
        var_y_vector_tmp = [mid_vax_y] + (linspace(mid_pos[1], obj_pos[1], int(nb_steps/2))).tolist()[1:]
        var_y_vector += var_y_vector_tmp
        
        var_z_vector = (linspace(eef_pos[2], mid_vax_z,
                        int(nb_steps/2))).tolist()
        var_z_vector = [round(pos, sim_param.round_value+2) for pos in var_z_vector]
        var_z_vector_tmp = [mid_vax_z] + [mid_pos[2] for i in range(len(var_z_vector),nb_steps)][1:]
        var_z_vector += var_z_vector_tmp
        
        traj = [[var_x_vector[i], var_y_vector[i], var_z_vector[i]] 
                 for i in range(len(var_x_vector))]
        traj_vector.append(traj)
        
        ''' Create diverse trajs '''
        tmp_div_traj_vector = create_diverse_trajs(traj, obj_pos, effect)
        traj_diverse_vector += tmp_div_traj_vector
    
    ''' Plot traj '''
    ax = plot_setup()
    plot_traj(ax, traj_vector)
        
#    ''' Plot new trajs '''
#    ax = plot_setup()
#    for curr_traj in traj_diverse_vector:
#        plot_traj_eef_obj(ax, curr_traj)
        
    plt.show()

    return traj_diverse_vector

''' 
Write the trajs dataset 
'''     
def write_dataset(traj_vector):    
    file = open(filename, 'w')
    for traj in traj_vector:
        for value in traj:
            file.write(str(value))
            file.write(',')
        file.write('\n')     

''' 
Read the trajs dataset 
'''     
def read_dataset(traj_vector):    
    lines = open(filename, 'r').readlines()
    traj_vector = []
    for line in lines:
        pos_rot_vector = line[:-2].split(',') ## remove final , and EOL
        traj = []
        for pos in range(0, len(pos_rot_vector), 12):
            current_x = float(pos_rot_vector[pos+0])
            current_y = float(pos_rot_vector[pos+1])
            current_z = float(pos_rot_vector[pos+2])
            traj.append([current_x,
                         current_y,
                         current_z])
        traj_vector.append(traj)
    
    return traj_vector

    
    
if __name__ == "__main__":
    
    filename = '../../../../a2l_exp_baxter_actions/src/generated_datasets/directed_dataset.csv'    
    
    if 1: ## create
    
        nb_diverse_trajs = 100
        traj_change = 0.005
        round_value = 2
        
        nb_init_traj = nb_diverse_trajs
        nb_steps = 10
        
        success = ros_services.call_restart_world("all")
        if not success:
            print("ERROR - restart_world failed")
        
        traj_vector = generate_dataset('right')
        res = write_dataset(traj_vector)
        
    else: ## plot 
        traj_vector = read_dataset(filename)
        ax = plot_setup()
        plot_traj(ax, traj_vector)
        
    