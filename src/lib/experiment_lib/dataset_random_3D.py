# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

import matplotlib.pyplot as plt
import random

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
import environment_dynamics as env
import environment_delta as delta
import environment_setup as setup
import discretize_effect as discr
import simulation_parameters as sim_param
import ros_services

from mpl_toolkits.mplot3d import Axes3D
import numpy as np     
from collections import OrderedDict


def plot_setup(obj_vector,
               init_pos_vector):
    
    # plot figure
    fig = plt.figure(figsize=(7,7))
    fig.clf()
    ax = Axes3D(fig)

    # plot initial position
    list_x_axis = init_pos_vector[0]
    list_y_axis = init_pos_vector[1]
    list_z_axis = init_pos_vector[2]
    ax.plot(list_x_axis,
            list_y_axis,
            list_z_axis,
            'o',
            color='red',
            markersize = 5)
    
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
             pos_cube[2] - 0.08/2, 
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

            
def get_environment_pos():
    
    object_vector = []
    
    ''' Get obj and eef states '''
    eef_pos = ros_services.call_get_eef_pose('left')
    eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos[0:3]]
    print("eef_pos", eef_pos)

    cube_pos = ros_services.call_get_model_state(sim_param.obj_name_vector[0])
    cube_pos = [round(pos, sim_param.round_value + 1) for pos in cube_pos[0:3]]
#    cube_pos[2] = round(cube_pos[2] + cube_height/2 + 0.015, 
#                        sim_param.round_value + 1)
    cube_pos[2] = -0.09
    object_vector.append(cube_pos)
    print('cube pos', cube_pos)
    
    return eef_pos, object_vector 


''' 
Write the trajs dataset 
'''     
def write_dataset(traj_vector):
#    filename = '../../../a2l_exp_baxter_actions/src/' + sim_param.generated_files_folder + 'random_dataset.csv'
#    if __name__ == '__main__':
    filename = '/home/maestre/git/a2l_exp_baxter_actions/src/generated_datasets/random_dataset.csv'
    
    file = open(filename, 'w')
    for traj in traj_vector:
        for value in traj:
            file.write(str(round(value,sim_param.round_value)))
            file.write(',')
        file.write('\n')

'''
Generate random trajs from each initial position
'''
def create_discr_trajs(nb_initial_pos, 
                       single_init_pos = False,
                       single_pos = 0):

    ''' Get all object pos'''
    obj_pos_dict = OrderedDict()
    for obj_name in sim_param.obj_name_vector:
        obj_pos = ros_services.call_get_model_state(obj_name)
        obj_pos = [round(pos, sim_param.round_value+1) for pos in obj_pos[0:3]]
        obj_pos_dict[obj_name] = obj_pos

    
    ''' Big circle '''      
    list_x_axis, list_y_axis, list_z_axis = \
        setup.gen_init_eef(nb_initial_pos,
                           sim_param.untucked_left_eef_pos[0] - obj_pos_dict[sim_param.obj_name_vector[0]][0],
                           obj_pos_dict[sim_param.obj_name_vector[0]])
    
    if __name__ == '__main__':
        ax = plot_setup([obj_pos_dict["cube"]],
                        [list_x_axis, 
                         list_y_axis, 
                         list_z_axis])
        ax.view_init(90,180) # top view
    
    ''' Generate the trajs '''
    it = 0
    step_length = sim_param.step_length
    step_options = [-step_length, 0, step_length]
    nb_max_box_touched_found = False
    nb_box_touched = 0
    delta_vector = []

    ## Nb of initial positions from where generate a traj
    if not single_init_pos:
        init_pos_range = range(0,len(list_x_axis))    
    else: ## if single_init_pos only generate trajs in pos single_pos
        init_pos_range = range(single_pos, single_pos + 1)     
    
    traj_vector = []
    while not nb_max_box_touched_found and it < 50000*2:
        ## for each initial position one traj is created each time        
#        for p in range(5,6):    
#        for p in range(0,1):
        for curr_init_pos in init_pos_range:

            ## compute each step of the trajectory, called delta
            x_i = [list_x_axis[curr_init_pos]]
            y_i = [list_y_axis[curr_init_pos]]
            z_i = [list_z_axis[curr_init_pos]]
            wp_vector = [list_x_axis[curr_init_pos],
                         list_y_axis[curr_init_pos],
                         list_z_axis[curr_init_pos],
                         1,1,1] ## fake eef orientation
            wp_vector += obj_pos_dict["cube"]
            wp_vector += [2,2,2] ## fake obj orientation                         
            
            nb_delta = 0
            obj_moved = False
            current_delta_vector = []
            effect = 'None'
            while nb_delta < sim_param.random_max_movs and \
                not obj_moved: # only N movs (delta) allowed
                
                current_x = x_i[-1]
                current_y = y_i[-1]
                current_z = z_i[-1]
                if sim_param.semi_random_trajs:
                    new_x = current_x + random.choice(step_options)
                    new_y = current_y + random.choice(step_options)
#                    new_z = current_z + random.choice(step_options)
                else:
                    new_x = current_x + random.uniform(-step_length,step_length)
                    new_y = current_y + random.uniform(-step_length,step_length)
#                    new_z = current_z + random.uniform(-step_length,step_length)   
                
                
                ############################################################################## TODO OJO !!!
                new_z = current_z
                    
                x_i.append(new_x)
                y_i.append(new_y)
                z_i.append(new_z)                                
                
                ## compute new box pos
                updated_obj_pos = env.compute_obj_pos(
                                        obj_pos_dict["cube"],
                                        [current_x, current_y, current_z],
                                        [new_x, new_y, new_z])
                updated_obj_pos = updated_obj_pos[1]

                wp_vector += [new_x, new_y, new_z, 1,1,1]
                wp_vector += updated_obj_pos
                wp_vector += [2,2,2] ## fake obj orientation
                
                obj_moved = True \
                    if updated_obj_pos != obj_pos_dict["cube"] else False
                        
                ## store current delta if there was a move
                if current_x != new_x or \
                    current_y != new_y:         
                    
                    current_delta = delta.Delta(
                                effect,
                                current_x,current_y,current_z,
                                new_x, new_y, new_z,
                                [obj_pos_dict["cube"][0], 
                                 obj_pos_dict["cube"][1],
                                 obj_pos_dict["cube"][2],
                                 updated_obj_pos[0], 
                                 updated_obj_pos[1],
                                 updated_obj_pos[2]])
#                                ,
#                                obj_moved)
                    current_delta_vector.append(current_delta)
#                    print(len(current_delta_vector))
                
                if obj_moved:
                    ## stop generating wps if max trajs inferred
                    nb_box_touched += 1
                    print('nb_box_touched', nb_box_touched)
                    if nb_box_touched == \
                        nb_initial_pos * len(sim_param.effect_values) * 3:
                        nb_max_box_touched_found = True
                    ## compute related effect                                            
                    effect = discr.compute_effect(obj_pos_dict["cube"],
                                                  updated_obj_pos)
#                    print(updated_obj_pos, effect)
                    for d in current_delta_vector:
                        d.set_effect(effect)
                    
                nb_delta += 1
            
            ## print traj
            if __name__ == "__main__" and obj_moved:
#                ax = plot_setup([obj_pos_dict["cube"]],
#                                        [list_x_axis, 
#                                         list_y_axis, 
#                                         list_z_axis])                
                ax.plot(x_i, y_i, z_i, '-')
                ax.plot(x_i, y_i, z_i, '*', c='grey')                
          
            ## only store trajs contacting the box
            if obj_moved :
                delta_vector = delta_vector + current_delta_vector
                traj_vector.append(wp_vector)
        it += 1

    if __name__ == "__main__":
        plt.show()
        
#    if write_dataset_bool:
    write_dataset(traj_vector)
    
    return delta_vector
    
'''
Test
'''
#if sim_param.test_random_dataset:
if __name__ == "__main__":
    write_dataset_bool = True
    
    create_discr_trajs(8, 
                       False, ## single_init_pos ?
                       2) ## single_pos