# -*- coding: utf-8 -*-
"""
@author: maestre
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#import random

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
#import ros_services
import simulation_parameters as sim_param
from numpy import linspace
import copy

from mpl_toolkits.mplot3d import Axes3D
#from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
#from scipy.linalg import norm

def plot_setup(obj_vector):
    
    # plot figure
    fig = plt.figure(figsize=(7,7))
    fig.clf()
    ax = Axes3D(fig)
    
    # robot
    robot_width = .2
    robot_height = .6
    robot_length = .4
    ax.bar3d(-robot_width/2, 
             -robot_length/2, 
             -robot_height/2, 
             robot_width, robot_length, robot_height, 
             color='red',
             alpha=0.2,
             edgecolor='none')                           
             
    # labels
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    
    pos_cube = obj_vector[0]
    tmp_obj_vector = [[pos_cube]]        
        
    plot_objects(ax, tmp_obj_vector)

    return ax
    
def plot_objects(ax, obj_vector):
    # box init_pos
    pos_cube = [obj_vector[0][0][0],
                obj_vector[0][0][1],
                obj_vector[0][0][2]]

    ax.bar3d(pos_cube[0] - 0.085/2, 
             pos_cube[1] - 0.07/2, 
             pos_cube[2] - 0.08, 
             [.085], [.07], [.08], 
             color='green',
             alpha=0.2,
             edgecolor='none')
             
    # limits
    obj_pos = pos_cube
    lim = .3
    ax.set_xlim3d([(obj_pos[0]-lim), (obj_pos[0]+lim)])
    ax.set_ylim3d([(obj_pos[1]-lim), (obj_pos[1]+lim)])
    ax.set_zlim3d([(obj_pos[2]-lim), (obj_pos[2]+lim)])
    
    ax.view_init(90,0) # top view
    

def plot_traj(ax, traj):
    
    color_vector = []
#    for effect,traj in traj_vector:        
    ## plot traj from init pos
    color = np.random.rand(3,1)
    color_vector.append(color)

    eef_pos_vector_x = [pos[0] for pos in traj]
    eef_pos_vector_y = [pos[1] for pos in traj]
    eef_pos_vector_z = [pos[2] for pos in traj]
    ax.plot(eef_pos_vector_x, 
            eef_pos_vector_y, 
            eef_pos_vector_z,
            '-')
#                color = 'black') #color)
    ax.plot(eef_pos_vector_x, 
            eef_pos_vector_y, 
            eef_pos_vector_z,
            '*',                                     
            markersize=3,
            c='grey')
                
    return color_vector

def create_diverse_trajs(traj,
                         obj_pos_vector,
                         effect):

        tmp_traj = [] ## [eef_pos eef_orien obj_pos obj_orien] = [float]
        
        zero_vector = [1,1,1]
        
        for traj_wp in traj:
            orig_traj_x = round(traj_wp[0], sim_param.round_value)
            orig_traj_y = round(traj_wp[1], sim_param.round_value)
            orig_traj_z = round(traj_wp[2], sim_param.round_value)
                        
            ## eef pos
            tmp_traj += [orig_traj_x, orig_traj_y, orig_traj_z]
            
            ## eef_orien                         
            tmp_traj += zero_vector
            
            ## obj pos and orientation
            tmp_traj += obj_pos_vector[0]
            tmp_traj += zero_vector                
            
        ## add displacement for final obj pos
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
            
        elif effect == 'far_right':
            displ_vector[0] += sim_param.obj_displacement
            displ_vector[1] -= sim_param.obj_displacement
        elif effect == 'far_left':
            displ_vector[0] += sim_param.obj_displacement
            displ_vector[1] += sim_param.obj_displacement
        elif effect == 'close_right':
            displ_vector[0] -= sim_param.obj_displacement 
            displ_vector[1] -= sim_param.obj_displacement
        elif effect == 'close_left':
            displ_vector[0] -= sim_param.obj_displacement
            displ_vector[1] += sim_param.obj_displacement
        tmp_obj_pos = [x+y for x,y in zip(obj_pos_vector[0], displ_vector)]
        tmp_traj += tmp_obj_pos
        tmp_traj += zero_vector        

            
        return [tmp_traj]       

    
def generate_dataset():


    ''' Positions '''
    pos_far_left = [cube_pos[0] + 0.2,
                     cube_pos[1] + 0.2,
                     cube_pos[2]]
                     
    pos_far_center = [cube_pos[0] + 0.2,
                     cube_pos[1],
                     cube_pos[2]]

    pos_far_right = [cube_pos[0] + 0.2,
                     cube_pos[1] - 0.2,
                     cube_pos[2]]
                     
    pos_center_left = [cube_pos[0],
                     cube_pos[1] + 0.2,
                     cube_pos[2]]
                     
    pos_center_right = [cube_pos[0],
                     cube_pos[1] - 0.2,
                     cube_pos[2]]

    pos_close_left = [cube_pos[0] - 0.2,
                     cube_pos[1] + 0.2,
                     cube_pos[2]]
                     
    pos_close_center = [cube_pos[0] - 0.2,
                     cube_pos[1],
                     cube_pos[2]]

    pos_close_right = [cube_pos[0] - 0.2,
                     cube_pos[1] - 0.2,
                     cube_pos[2]]

    traj_diverse_vector = []    
    
    ''' Left init pos '''
    traj_vector = []
    traj_vector_final = []
    left_init_pos = copy.copy(cube_pos)
    left_init_pos[1] += 0.2
    
    ## right effect
    effect = 'right'
    main_traj = [left_init_pos]                    
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))

    ## close-right effect 
    effect = 'close_right'
    main_traj = [left_init_pos]                    
    main_traj.append(pos_far_left)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))

    ## close effect 
    effect = 'close'
    main_traj = [left_init_pos]                    
    main_traj.append(pos_far_left)
    main_traj.append(pos_far_center)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))
    
    ## close-left effect
    effect = 'close_left'
    main_traj = [left_init_pos]                    
    main_traj.append(pos_far_left)
    main_traj.append(pos_far_center)
    main_traj.append(pos_far_right)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))

    ## left effect
    effect = 'left'
    main_traj = [left_init_pos]                    
    main_traj.append(pos_far_left)
    main_traj.append(pos_far_center)
    main_traj.append(pos_far_right)
    main_traj.append(pos_center_right)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj)) 

    ## far-right effect 
    effect = 'far_right'
    main_traj = [left_init_pos]                    
    main_traj.append(pos_close_left)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))
    
    ## far effect
    effect = 'far'
    main_traj = [left_init_pos]                    
    main_traj.append(pos_close_left)
    main_traj.append(pos_close_center)
    main_traj.append(cube_pos)    
    traj_vector.append((effect, main_traj))
    
    ## far-left effect
    effect = 'far_left'
    main_traj = [left_init_pos]                    
    main_traj.append(pos_close_left)
    main_traj.append(pos_close_center)
    main_traj.append(pos_close_right)
    main_traj.append(cube_pos)    
    traj_vector.append((effect, main_traj)) 


    ''' Right init pos '''
#    traj_vector = []
#    traj_vector_final = []
    right_init_pos = copy.copy(cube_pos)
    right_init_pos[1] -= 0.2
    
    ## left effect
    effect = 'left'
    main_traj = [right_init_pos]                    
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))

    ## close-left effect 
    effect = 'close_left'
    main_traj = [right_init_pos]                    
    main_traj.append(pos_far_right)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))

    ## close effect 
    effect = 'close'
    main_traj = [right_init_pos]                    
    main_traj.append(pos_far_right)
    main_traj.append(pos_far_center)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))
    
    ## close-right effect
    effect = 'close_right'
    main_traj = [right_init_pos]                    
    main_traj.append(pos_far_right)
    main_traj.append(pos_far_center)
    main_traj.append(pos_far_left)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))

    ## left effect
    effect = 'left'
    main_traj = [right_init_pos]                    
    main_traj.append(pos_far_right)
    main_traj.append(pos_far_center)
    main_traj.append(pos_far_left)
    main_traj.append(pos_center_left)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj)) 

    ## far-lef effect 
    effect = 'far_left'
    main_traj = [right_init_pos]                    
    main_traj.append(pos_close_right)
    main_traj.append(cube_pos)
    traj_vector.append((effect, main_traj))
    
    ## far effect
    effect = 'far'
    main_traj = [right_init_pos]                    
    main_traj.append(pos_close_right)
    main_traj.append(pos_close_center)
    main_traj.append(cube_pos)    
    traj_vector.append((effect, main_traj))
    
    ## far-right effect
    effect = 'far_right'
    main_traj = [right_init_pos]                    
    main_traj.append(pos_close_right)
    main_traj.append(pos_close_center)
    main_traj.append(pos_close_left)
    main_traj.append(cube_pos)    
    traj_vector.append((effect, main_traj)) 

    
    ''' Split each traj on small segments '''
    nb_segments = 5
    for effect, main_traj in traj_vector:
        var_x_vector = []
        var_y_vector = []
        var_z_vector = []        
        for pos in range(len(main_traj)-1):    
            if main_traj[pos][0] != main_traj[pos+1][0]: ## X
                tmp_sector_x = (linspace(main_traj[pos][0],
                                      main_traj[pos+1][0],
                                      nb_segments)).tolist()
            else:
                tmp_sector_x = [main_traj[pos][0] for val in range(nb_segments)]
            var_x_vector += tmp_sector_x
            
            if main_traj[pos][1] != main_traj[pos+1][1]: ## Y
                tmp_sector_y = (linspace(main_traj[pos][1],
                                      main_traj[pos+1][1],
                                      nb_segments)).tolist()
            else:
                tmp_sector_y = [main_traj[pos][1] for val in range(nb_segments)]
            var_y_vector += tmp_sector_y;
            
            if main_traj[pos][2] != main_traj[pos+1][2]: ## Z                   
                tmp_sector_z = (linspace(main_traj[pos][2],
                                      main_traj[pos+1][2],
                                      nb_segments)).tolist()
            else:
                tmp_sector_z = [main_traj[pos][2] for val in range(nb_segments)]
            var_z_vector += tmp_sector_z;                          

        traj = [[var_x_vector[i], var_y_vector[i], var_z_vector[i]] 
                 for i in range(len(var_x_vector))]
        traj_vector_final.append((effect, traj))
    
    ''' Create diverse trajs '''
    obj_vector = [cube_pos]
    for effect, traj in traj_vector_final:
        print(effect)
        tmp_div_traj_vector = create_diverse_trajs(traj, 
                                                   obj_vector,
                                                   effect)
        traj_diverse_vector += tmp_div_traj_vector
    
        ''' Plot traj '''
        ax = plot_setup(obj_vector)
        plot_traj(ax, traj)
            
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
def read_dataset(filename):    
    lines = open(filename, 'r').readlines()    
    traj_vector = []
    obj_vector = []
    for line in lines:
        pos_rot_vector = line[:-2].split(',') ## remove final , and EOL
        traj = []
        obj = []
        for pos in range(0, len(pos_rot_vector), block_of_info):
            current_x = float(pos_rot_vector[pos+0])
            current_y = float(pos_rot_vector[pos+1])
            current_z = float(pos_rot_vector[pos+2])
            traj.append([current_x,
                         current_y,
                         current_z])
            for curr_obj_id in range(1,nb_objects+1):
                obj_pos_rot = pos_rot_vector[pos+6*curr_obj_id : pos+6*curr_obj_id + 6]
                obj_pos_rot = [float(value) for value in obj_pos_rot]
                obj.append(obj_pos_rot)
                
        traj_vector.append(traj)
        obj_vector.append(obj)
    
    return traj_vector, obj_vector

'''
a
'''      
if __name__ == "__main__":

    filename = '../../../../a2l_exp_baxter_actions/src/generated_datasets/directed_dataset.csv'    

    obj_name_vector = ['cube']
    cube_pos = sim_param.first_obj_pos
    nb_objects = len(obj_name_vector)
    block_of_info = 6 + 6*nb_objects
    
    create = True
    
    if create: ## create
        print('GENERATING DATASET')
        round_value = 3
        nb_steps = 3

        traj_vector = generate_dataset()
        res = write_dataset(traj_vector)
        
    else: ## plot
        print('VISUALIZING DATASET')
        traj_vector, obj_vector = read_dataset(filename)
        ax = plot_setup(obj_vector)
        color_vector = plot_traj(ax, traj_vector)
        plot_objects(ax, obj_vector)
        
    