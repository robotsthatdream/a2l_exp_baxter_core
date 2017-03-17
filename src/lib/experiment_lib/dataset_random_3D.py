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
#import numpy as np     
from collections import OrderedDict
import rospy
import baxter_interface
from scipy.spatial import distance

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

    ax.view_init(90,0) # top view    
             
    # labels
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    
    pos_cube = obj_vector[0]
    tmp_obj_vector = [[pos_cube]]        
        
    plot_objects(ax, tmp_obj_vector)

    ## limits
    lim = .3
    ax.set_xlim3d([pos_cube[0]-lim, pos_cube[0]+lim])
    ax.set_ylim3d([pos_cube[1]-lim, pos_cube[1]+lim])
    ax.set_zlim3d([pos_cube[2]-lim, pos_cube[2]+lim])   

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
def write_dataset(traj_vector,
                  delta_vector):
#    filename = '../../../a2l_exp_baxter_actions/src/' + sim_param.generated_files_folder + 'random_dataset.csv'
#    if __name__ == '__main__':
#    filename = '/home/maestre/git/a2l_exp_baxter_actions/src/generated_datasets/random_dataset.csv'
    
    filename = sim_param.generated_datasets_folder + 'random_dataset.csv'    
    file = open(filename, 'w')
    for traj in traj_vector:
        for value in traj:
            file.write(str(round(value,sim_param.round_value)))
            file.write(',')
        file.write('\n')
        
    filename = sim_param.generated_datasets_folder + 'random_dataset_deltas.csv'    
    file = open(filename, 'w')
    for curr_delta in delta_vector:
        file.write(curr_delta.get_effect())
        file.write(',')
        file.write(str(curr_delta.get_wp_init().get_x()))
        file.write(',')
        file.write(str(curr_delta.get_wp_init().get_y()))
        file.write(',')
        file.write(str(curr_delta.get_wp_init().get_z()))
        file.write(',')
        file.write(str(curr_delta.get_wp_final().get_x()))
        file.write(',')
        file.write(str(curr_delta.get_wp_final().get_y()))
        file.write(',')
        file.write(str(curr_delta.get_wp_final().get_z()))
        file.write(',')
        file.write(str(curr_delta.get_obj_init(0)[0]))
        file.write(',')
        file.write(str(curr_delta.get_obj_init(0)[1]))
        file.write(',')
        file.write(str(curr_delta.get_obj_init(0)[2]))
        file.write(',')
        file.write(str(curr_delta.get_obj_final(0)[0]))
        file.write(',')
        file.write(str(curr_delta.get_obj_final(0)[1]))
        file.write(',')
        file.write(str(curr_delta.get_obj_final(0)[2]))
        file.write(',')
        file.write('\n')
        
'''
Generate random trajs from each initial position
'''
def create_discr_trajs(nb_initial_pos, 
                       single_init_pos = False,
                       single_pos = 0):

    ''' Restart scenario '''         
    success = ros_services.call_restart_world("all")
    if not success:
        print("ERROR - restart_world failed")
    
    ''' Generate the trajs '''
    it = 0
    step_length = sim_param.step_length
    step_options = [-step_length, 0, step_length]
    nb_max_box_touched_found = False
    nb_box_touched = 0
    delta_vector = []

    ## Nb of initial positions from where generate a traj
    if not single_init_pos:
        init_pos_range = range(0,sim_param.nb_min_init_pos)
    else: ## if single_init_pos only generate trajs in pos single_pos
        init_pos_range = range(single_pos, single_pos + 1)
    
    ## load eef interface
    rospy.init_node('left_gripper_node')
    left_gripper = baxter_interface.Gripper('left')
    rospy.sleep(1)
    
    thrs = sim_param.obj_moved_threshold
    traj_vector = []
    while not nb_max_box_touched_found and it < 5000*1:
        ## for each initial position one traj is created each time        
#        for p in range(5,6):    
#        for p in range(0,1):
        for curr_init_pos in init_pos_range:

            ''' Get object pos'''
            obj_pos_dict = OrderedDict()
            obj_pos = ros_services.call_get_model_state("cube")
            obj_pos = [round(pos, sim_param.round_value+1) for pos in obj_pos[0:3]]
            obj_pos_dict["cube"] = obj_pos
            
            ''' Big circle '''      
            list_x_axis, list_y_axis, list_z_axis = \
                setup.gen_init_eef(nb_initial_pos,
                                   sim_param.radio,
                                   obj_pos_dict[sim_param.obj_name_vector[0]])                                                                      
                                   
#            if __name__ == '__main__':
#                ax = plot_setup([obj_pos_dict["cube"]],
#                                [list_x_axis, 
#                                 list_y_axis, 
#                                 list_z_axis])
#                ax.view_init(90,180) # top view                                   


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
            sim_obj_moved = False
            real_obj_moved = False
            current_delta_vector = []
            real_effect = None
            while nb_delta < sim_param.random_max_movs and \
                not real_obj_moved: # only N movs (delta) allowed

                print(curr_init_pos, nb_delta)
                
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
                sim_final_obj_pos = env.compute_obj_pos(
                                        obj_pos_dict["cube"],
                                        [current_x, current_y, current_z],
                                        [new_x, new_y, new_z])
                sim_final_obj_pos = sim_final_obj_pos[1]

                wp_vector += [new_x, new_y, new_z, 1,1,1]
                wp_vector += sim_final_obj_pos
                wp_vector += [2,2,2] ## fake obj orientation
                
                sim_obj_moved = True \
                    if sim_final_obj_pos != obj_pos_dict["cube"] else False
                        
                ## store current delta if there was a move
                if current_x != new_x or \
                    current_y != new_y:         
                    
                    current_delta = delta.Delta(
                                sim_obj_moved,
                                current_x,current_y,current_z,
                                new_x, new_y, new_z,
                                [obj_pos_dict["cube"][0], 
                                 obj_pos_dict["cube"][1],
                                 obj_pos_dict["cube"][2],
                                 sim_final_obj_pos[0], 
                                 sim_final_obj_pos[1],
                                 sim_final_obj_pos[2]])
                    current_delta_vector.append(current_delta)            
            
                ## check if movement in real robot
                if sim_obj_moved:          

                    ## eef to init pos
                    ''' Update eef pos'''
                    init_pos_coord = [x_i[0], y_i[0], z_i[0]]
                    success = ros_services.call_move_to_initial_position(init_pos_coord) 
                    if not success:
                        print("ERROR - call_move_to_initial_position failed")                                   
                    eef_pos = ros_services.call_get_eef_pose('left')
                    eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos[0:3]]
                    left_gripper.close()

                    initial_obj_pos = obj_pos_dict["cube"]
                    
                    ## execute real mov to get real effect
                    simulated_traj = zip(x_i, y_i, z_i)
                    simulated_traj_vector = [i for el in simulated_traj for i in el] ## [float]
                    res_exec = ros_services.call_trajectory_motion(sim_param.feedback_window, 
                                                               simulated_traj_vector)
                    ''' Get object pos'''
                    obj_pos_dict = OrderedDict()
                    obj_pos = ros_services.call_get_model_state("cube")
                    obj_pos = [round(pos, sim_param.round_value+1) for pos in obj_pos[0:3]]                    
                    obj_pos_dict["cube"] = obj_pos
                    real_final_obj_pos = obj_pos

                    real_obj_moved = \
                        (abs(initial_obj_pos[0] - real_final_obj_pos[0]) +
                         abs(initial_obj_pos[1] - real_final_obj_pos[1]) +
                         abs(initial_obj_pos[2] - real_final_obj_pos[2])) >= thrs

                    ## store current delta if there was a real contact with the box
                    if real_obj_moved:                                    
                        real_effect = discr.compute_effect(initial_obj_pos,
                                                           real_final_obj_pos)                                                                                                       
                        print("real_effect", real_effect)
                        
                        if real_effect != None:
                            ## stop generating wps if max trajs inferred
                            nb_box_touched += 1
                            print('nb_box_touched', nb_box_touched)
                            if nb_box_touched == \
                                nb_initial_pos * len(sim_param.effect_values) * 1:
                                nb_max_box_touched_found = True
                                
                            ## print traj
                            if __name__ == "__main__" and sim_obj_moved:
                                ax = plot_setup([initial_obj_pos],
                                                [list_x_axis, 
                                                 list_y_axis, 
                                                 list_z_axis])             
                            
                            ax.plot(x_i, y_i, z_i, '-')
                            ax.plot(x_i, y_i, z_i, '*', c='grey')    
                            
                            for d in current_delta_vector:
                                d.set_effect(real_effect)
                            
                            delta_vector = delta_vector + current_delta_vector
                            traj_vector.append(wp_vector)
                        
                            ## update cube pos
                            if distance.euclidean(real_final_obj_pos, 
                                                   sim_param.first_obj_pos) > \
                                                   sim_param.obj_too_far_distance:        
                                print('-------------> UPDATING CUBE POSITION!')
                                
                                ## compute new obj pos
                                new_obj_pos = sim_param.first_obj_pos
                                new_obj_pos = [new_obj_pos[0] + random.uniform(-sim_param.new_obj_pos_dist,
                                                                               sim_param.new_obj_pos_dist),
                                               new_obj_pos[1] + random.uniform(-sim_param.new_obj_pos_dist,
                                                                               sim_param.new_obj_pos_dist),
                                               new_obj_pos[2]]
                                new_obj_pos = [round(poss, sim_param.round_value) for poss in new_obj_pos]
                                
                                ## move obj to new pos    
                                success = ros_services.call_restart_world("object",
                                                                          sim_param.obj_name_vector[0],
                                                                          new_obj_pos)                        
                                if not success:
                                    print("ERROR - restart_world failed for object", sim_param.obj_name_vector[0])                         
                    
                nb_delta += 1                
        it += 1

    if __name__ == "__main__":
        plt.show()
        
#    if write_dataset_bool:
    write_dataset(traj_vector,
                  delta_vector)
    
    return delta_vector
    
'''
Test
'''
#if sim_param.test_random_dataset:
if __name__ == "__main__":
#    write_dataset_bool = True
    
    create_discr_trajs(sim_param.nb_min_init_pos, 
                       False, ## single_init_pos ?
                       2) ## single_pos