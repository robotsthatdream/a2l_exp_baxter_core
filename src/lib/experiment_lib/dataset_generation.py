#!/usr/bin/env python

"""
@author: maestre
"""
from __future__ import print_function

import random
#import numpy as np

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
import dataset_directed_extended as dde
import dataset_random_3D as dr
import environment_dynamics as env
import environment_delta as delta
import discretize_effect as discr
import inference_ros as infer_ros
import inf_traj_series_dataset_classes as dataset_series_classes
import simulation_parameters as sim_param
import ros_services

import matplotlib.pyplot as plt
import environment_setup as setup
from mpl_toolkits.mplot3d import Axes3D
from collections import OrderedDict
import copy
    
import scipy.spatial.distance as d
from math import sqrt

'''
Create directed or random dataset
'''
def create_dataset(dataset_type, nb_initial_pos):
#    if dataset_type == 'directed':
#        print('DIRECTED dataset generated')
#        return dd.create_discr_trajs(nb_initial_pos)        
#    el
    if dataset_type == 'directed':
        print('DIRECTED dataset generated')
        return dde.create_discr_trajs(nb_initial_pos)        
        
    elif dataset_type == 'random':
        print('RANDOM dataset generated')
        return dr.create_discr_trajs(nb_initial_pos)        
    else:
        print('ERROR - create_dataset - wrong dataset_type value')           
        sys.exit()  


'''
Read deltas dataset
'''
def read_delta_dataset(filename):

    delta_vector = []
    lines = open(filename, 'r').readlines()
    for line in lines:        
        values_vector = line[:-2].split(',') ## remove final , and EOL
        current_delta = delta.Delta(
            values_vector[0],
            float(values_vector[1]),float(values_vector[2]),float(values_vector[3]),
            float(values_vector[4]),float(values_vector[5]),float(values_vector[6]),
            [float(values_vector[7]), 
             float(values_vector[8]),
             float(values_vector[9]),
             float(values_vector[10]), 
             float(values_vector[11]),
             float(values_vector[12])])
        delta_vector.append(current_delta)
#            print("in discretize_trajs :")
#            current_delta.print_me()
    
    return delta_vector

'''
Read trajs of externally generated RAW dataset
traj = [EP EO OP1 OO1 OP2 OO2 etc]
EP = x y z
EO = roll pitch yaw
OPi = x y z
OOi = roll pitch yaw
'''
def read_dataset(filename):
#    mean_move_size = 0
    nb_moves = 0
    delta_vector = []
    lines = open(filename, 'r').readlines()
    for line in lines:        
        current_delta_vector = []
        pos_rot_vector = line[:-2].split(',') ## remove final , and EOL
        nb_obj = len(sim_param.obj_name_vector) ## expected
#        nb_dataset_obj = sim_param.dataset_nb_objects ## actual
        
        obj_initial_pos = [float(pos_rot_vector[6]),
                           float(pos_rot_vector[7]),
                           float(pos_rot_vector[8])]
        obj_final_pos = [float(pos_rot_vector[-6 -6*(nb_obj-1)]),
                         float(pos_rot_vector[-5 -6*(nb_obj-1)]),
                         float(pos_rot_vector[-4 -6*(nb_obj-1)])]
        obtained_effect = discr.compute_effect(obj_initial_pos,
                                                 obj_final_pos)
        
        related_info_size = 6 + 6*nb_obj
        for pos in range(0, len(pos_rot_vector)-related_info_size, related_info_size):
            current_x = float(pos_rot_vector[pos+0])
            current_y = float(pos_rot_vector[pos+1])
            current_z = float(pos_rot_vector[pos+2])
            
            next_x = float(pos_rot_vector[pos+related_info_size+0])
            next_y = float(pos_rot_vector[pos+related_info_size+1])
            next_z = float(pos_rot_vector[pos+related_info_size+2])
            
#            mean_move_size += d.euclidean([current_x, current_y, current_z],
#                                          [next_x, next_y, next_z])
            nb_moves += 1

            current_next_obj_pos_vector = []
            for curr_obj_id in range(1,len(sim_param.obj_name_vector)+1):
                
                current_obj_pos_x = float(pos_rot_vector[pos+6*curr_obj_id+0])
                current_obj_pos_y = float(pos_rot_vector[pos+6*curr_obj_id+1])
                current_obj_pos_z = float(pos_rot_vector[pos+6*curr_obj_id+2])
                
                next_obj_pos_x = float(pos_rot_vector[pos+related_info_size+6*curr_obj_id+0])
                next_obj_pos_y = float(pos_rot_vector[pos+related_info_size+6*curr_obj_id+1])
                next_obj_pos_z = float(pos_rot_vector[pos+related_info_size+6*curr_obj_id+2])            

#                current_next_obj_pos = [current_obj_pos_x, current_obj_pos_y,current_obj_pos_z,
#                                        next_obj_pos_x, next_obj_pos_y, next_obj_pos_z]
#                current_next_obj_pos_vector.append(current_next_obj_pos)
                current_next_obj_pos_vector += [current_obj_pos_x, current_obj_pos_y,current_obj_pos_z,
                                                    next_obj_pos_x, next_obj_pos_y, next_obj_pos_z]
                
            current_delta = delta.Delta(
                obtained_effect,
                current_x,current_y,current_z,
                next_x, next_y,next_z,
                current_next_obj_pos_vector)
         
            current_delta_vector.append(current_delta)
        delta_vector += current_delta_vector
        
#    mean_move_size /= nb_moves
#    sim_param.step_length = round(mean_move_size, sim_param.round_value)
    print('Move step length:', sim_param.step_length)
    return delta_vector

'''
Compute cumulated performance value for each dataset type
'''
def compute_cumulated_perf(current_dataset,
                           #dataset_stats_vector,
                           learn_algo_vector,
                           initial_pos_vector):
    dataset_perf_value_vector = []
#    for current_dataset in dataset_stats_vector:

    ## Classes to store the inferred trajs cumulated perf   
    dataset_name = current_dataset.get_dataset_name()
    current_dataset_cumulated_perf_class = \
        dataset_series_classes.Dataset_stats(dataset_name)
    for current_algo in learn_algo_vector:
        current_perf_value_dict = \
            infer_ros.compute_perf_value_for_init_pos(
                    current_dataset,
                    current_algo,
                    initial_pos_vector)  
        current_dataset_cumulated_perf_class.add_algo_results(
            current_algo, current_perf_value_dict)
    ## Store them
    dataset_perf_value_vector.append(
        current_dataset_cumulated_perf_class)    
    return dataset_perf_value_vector

''' Plot simulated extended trajectory '''
def plot_sim_extended_traj(obj_pos,
                           nb_initial_pos,
                           traj_vector_x,
                           traj_vector_y,
                           traj_vector_z):
        
    # plot figure
    fig = plt.figure(figsize=(7,7))
    fig.clf()
    ax = Axes3D(fig)
    
    # limits
    lim = .3
    ax.set_xlim3d([obj_pos[0]-lim, obj_pos[0]+lim])
    ax.set_ylim3d([obj_pos[1]-lim, obj_pos[1]+lim])
    ax.set_zlim3d([obj_pos[2]-lim, obj_pos[2]+lim])
    
    ## plot object
    pos_cube = obj_pos
    ax.bar3d(pos_cube[0] - 0.085/2, 
             pos_cube[1] - 0.07/2, 
             pos_cube[2] - 0.08/2, 
             [.085], [.07], [.08], 
             color='green',
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
             color='red',
             alpha=0.2,
             edgecolor='none')
    
    # labels
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')            
    
    ax.view_init(90,0) # top view
             
    ## plot the big circle points         
    list_x_axis, list_y_axis, list_z_axis = \
    setup.gen_init_eef(
        nb_initial_pos,
        sim_param.radio,
        obj_pos)
    ax.plot(list_x_axis,list_y_axis,'.',c='grey')   

    ## plot trajs
    ax.plot(traj_vector_x,
            traj_vector_y,
            traj_vector_z,
            '-*',
            linewidth=8)  
                        
    plt.show()

'''
Extend dataset generating new deltas based on successful trajs
'''
def extend_dataset(current_dataset_stats, 
                   current_nb_initial_pos, ## 8
                   learn_algo_vector,
                   perf_value_changes_vector, ## [0, 3, 1, 0])
                   dataset_size,
                   left_gripper_interface):
    
    for current_algo in learn_algo_vector:
        current_algo_stats_class = \
            current_dataset_stats.get_algo_results(current_algo)
            
        current_dataset_size_dict = \
            current_algo_stats_class.get_results_dict()
        current_dataset_size_class = \
            current_dataset_size_dict[current_nb_initial_pos]
        current_trajs_dict = \
            current_dataset_size_class.get_indiv_traj_info()
            
        ## for each initial position pos and effect generate new trajs
        new_delta_vector = []
        new_random_directed_trajs_vector = []
        for current_init_pos in range(current_nb_initial_pos):
            
            ## identify actions to learn
            actions_to_learn_vector = []
            for tmp_effect in sim_param.effect_values:                
                current_traj_class = \
                    current_trajs_dict[current_init_pos, tmp_effect]
                current_traj_res = current_traj_class.get_traj_res()
                if current_traj_res != 'success':
                    actions_to_learn_vector.append(tmp_effect)
            
            if len(actions_to_learn_vector) != len(sim_param.effect_values):
                ## generate new trajs (deltas) based on previous contacts
                for expected_effect in sim_param.effect_values:
                    print('actions_to_learn_vector', actions_to_learn_vector)
                    
                    current_traj_class = \
                        current_trajs_dict[current_init_pos, expected_effect]
                    current_traj_res = current_traj_class.get_traj_res()

                    if current_traj_res == 'success' or current_traj_res == 'false_pos':
                        current_traj = \
                            current_traj_class.get_eef_traj_vector()
                        current_delta = \
                            current_traj_class.get_delta_vector()                    
                        current_new_delta_vector, \
                        actions_to_learn_vector = \
                            extend_trajectory(current_traj, 
                                              current_delta,
                                              current_nb_initial_pos,
                                              current_init_pos, 
                                              expected_effect,
                                              dataset_size,
                                              current_trajs_dict,
                                              actions_to_learn_vector,
                                              left_gripper_interface)
                        new_delta_vector += current_new_delta_vector                                         
                        print('len new deltas', len(current_new_delta_vector),
                              'init_pos - effect :',
                              current_init_pos, expected_effect)
            else:
                ## generate new random trajs for the init pos
                print('---------> Babbling for init_pos :', current_init_pos)
                current_new_delta_vector = dr.create_discr_trajs(sim_param.nb_min_init_pos, ## nb init pos
                                                                 left_gripper_interface,
                                                                 write_dataset_bool = False,
                                                                 single_init_pos = True,
                                                                 single_pos = current_init_pos)
                new_delta_vector += current_new_delta_vector
                print('len new deltas', len(current_new_delta_vector))
                
                                              
    print('Total delta extension :', len(new_delta_vector))
    return new_delta_vector, new_random_directed_trajs_vector

'''
Given a trajectory (init_pos-effect) tries to extend it with new moves
'''
def extend_trajectory(current_traj_vector, ## [WPs]
                      current_delta_vector, ## [delta_class]
                      nb_initial_pos, ## 8
                      current_init_pos,  ## 0,1,2,3,..., 7
                      expected_effect,
#                      new_random_directed_trajs, ## bool
                      dataset_size,
                      current_trajs_dict,
                      actions_to_learn_vector,
                      left_gripper_interface):            

    print('\n\n------------------->', current_init_pos, expected_effect)

    ## move arm up
    eef_pos = ros_services.call_get_eef_pose('left')
    eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos[0:3]]
    pos_up = copy.copy(eef_pos)
    pos_up[2] += 0.2
    success = ros_services.call_move_to_initial_position(pos_up) 
    if not success:
        print("ERROR - extend dataset failed")

    initial_obj_pos = current_delta_vector[0].get_obj_init()
    init_eef_pos = [current_delta_vector[0].get_wp_init().get_x(),
                    current_delta_vector[0].get_wp_init().get_y(),
                    current_delta_vector[0].get_wp_init().get_z()]
    
    ## move obj to new pos    
    success = ros_services.call_restart_world("object",
                                              sim_param.obj_name_vector[0],
                                              initial_obj_pos)    
    step_length = sim_param.step_length
    new_delta_trajs_vector = []
    nb_trajs_found = 0        
    
    ''' generate max N new trajs '''
    thrs = sim_param.obj_moved_threshold
    nb_new_traj = 0
    counter = 0    
    while nb_new_traj < sim_param.extend_max_trajs and counter < 10000:
        ## for (almost) each WP of the traj        
        curr_wp = 1
            
        ''' For each WP '''
        while curr_wp < len(current_traj_vector):                     
            
            ## new tmp traj from the beginning to this WP            
            tmp_traj = current_traj_vector[:-curr_wp]
            tmp_delta = current_delta_vector[:-curr_wp]
            
            traj_vector_x = [v[0] for v in tmp_traj]
            traj_vector_y = [v[1] for v in tmp_traj]
            traj_vector_z = [v[2] for v in tmp_traj]
            nb_new_delta = 0
            sim_obj_moved = False
            real_obtained_effect = ''
            extended_delta_vector = tmp_delta
                        
            ''' Generate new traj from a WP'''
            ## extend with new random moves from the closest to the more far one
            ## the more far is the WP respect the object, the more deltas can generate
            while nb_new_delta < (sim_param.extend_max_movs + (curr_wp-1)) and \
                  not sim_obj_moved:
                      
                current_x = traj_vector_x[-1]
                current_y = traj_vector_y[-1]
                current_z = traj_vector_z[-1]
                
                if sim_param.semi_random_trajs:
                    var_x = random.choice([-step_length, 0, step_length])
                    if var_x != 0:
                        var_y = 0
                    else:
                        var_y = random.choice([-step_length, 0, step_length])
#                    new_z = round(current_z + random.choice(step_options), sim_param.round_value)
                else:
                    var_x = random.uniform(-step_length/2,step_length/2)
                    var_y = random.uniform(-step_length/2,step_length/2)
#                    new_z = round(current_z + random.uniform(-step_length,step_length), sim_param.round_value)
#                new_x = round(current_x + var_x, sim_param.round_value)
#                new_y = round(current_y + var_y, sim_param.round_value)
                    
                ## normalize mov size
                var_vector = [var_x, var_y]
                dims_vector = 0               
                for value in var_vector:
                    if value != 0:                    
                        dims_vector += 1
                if dims_vector == 0:
                    dims_vector = 1
                dims_vector = 1
                var_vector = [round(value/sqrt(dims_vector), sim_param.round_value)
                                for value in var_vector]                                                
                new_x = round(current_x + var_vector[0], sim_param.round_value)
                new_y = round(current_y + var_vector[1], sim_param.round_value)                    

                ############################################################################## TODO OJO !!!
                new_z = current_z                    
                    
                traj_vector_x.append(new_x)
                traj_vector_y.append(new_y)
                traj_vector_z.append(new_z)
                
#                print("\nDelta :", var_vector[0], var_vector[1],
#                      'Norm:', round(d.euclidean([0, 0],
#                                                 [var_vector[0], var_vector[1]]), 3))
#                print('Previous EEF pos :', 
#                      current_x, current_y, current_z)
#                print('New EEF pos :', 
#                      new_x, new_y, new_z)
                
                ## compute new box pos
                sim_initial_obj_pos = initial_obj_pos #obj_pos_dict['cube']
                sim_obj_moved, sim_final_obj_pos = \
                    env.compute_obj_pos(sim_initial_obj_pos,
                                        [current_x,current_y,current_z],
                                        [new_x, new_y, new_z])
                        
                ## store current delta
                current_delta = delta.Delta(
                    '', ## effect
                    current_x,current_y,current_z,
                    new_x, new_y, new_z,
                    [sim_initial_obj_pos[0], sim_initial_obj_pos[1], sim_initial_obj_pos[2],
                     sim_final_obj_pos[0], sim_final_obj_pos[1], sim_final_obj_pos[2]])                                
                extended_delta_vector.append(current_delta)                                                  
                            
                nb_new_delta += 1
                ## end generate new deltas
            curr_wp += 1
            
            if sim_obj_moved:
                if var_x == 0 or var_y == 0 : 
                    ## replicate last move
                    traj_vector_x.append(round(new_x + var_x, sim_param.round_value))
                    traj_vector_y.append(round(new_y + var_y, sim_param.round_value))
                    traj_vector_z.append(new_z)
                    
                    replicated_delta = delta.Delta(
                        '',
                        new_x, new_y, new_z,
                        new_x + var_x, new_y + var_y, new_z,
                        [sim_initial_obj_pos[0], sim_initial_obj_pos[1], sim_initial_obj_pos[2],
                         sim_final_obj_pos[0], sim_final_obj_pos[1], sim_final_obj_pos[2]])                                
                    extended_delta_vector.append(replicated_delta)                
                    nb_new_delta += 1
                    
                ## compute related effect
                sim_obtained_effect = discr.compute_effect(sim_initial_obj_pos,
                                                           sim_final_obj_pos)
                                                                           
                if sim_obtained_effect in actions_to_learn_vector:    
                    print('\nsim_obtained_effect', sim_obtained_effect)
                    print('--> real - extending for', current_init_pos, sim_obtained_effect) 
                    
                    ## print simulated extended traj
                    print(current_init_pos, expected_effect, 
                          "- WP", curr_wp, "- extension", nb_new_delta)
                    plot_sim_extended_traj(initial_obj_pos,
                                           nb_initial_pos,
                                           traj_vector_x,
                                           traj_vector_y,
                                           traj_vector_z)
                
                    ''' Update eef pos'''
                    init_pos_coord = init_eef_pos                    
                    success = ros_services.call_move_to_initial_position(init_pos_coord) 
                    if not success:
                        print("ERROR - call_move_to_initial_position failed")                                   
                    eef_pos = ros_services.call_get_eef_pose('left')
                    eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos[0:3]]
                    left_gripper_interface.close()
    
                    real_initial_obj_pos = initial_obj_pos #obj_pos_dict["cube"]
                    
                    ## execute real mov to get real effect
                    simulated_traj_vector = zip(traj_vector_x,
                                                traj_vector_y,
                                                traj_vector_z)
                    simulated_traj_vector = [i for el in simulated_traj_vector for i in el] ## [float]
                    res_exec = ros_services.call_trajectory_motion(sim_param.feedback_window,   
                                                                   simulated_traj_vector)
                    ''' Get object pos'''
                    obj_pos_dict = OrderedDict()
                    obj_pos = ros_services.call_get_model_state("cube")
                    obj_pos = [round(pos, sim_param.round_value) for pos in obj_pos[0:3]]                    
                    obj_pos_dict["cube"] = obj_pos
                    real_final_obj_pos = obj_pos
    
                    real_obj_moved = \
                        (abs(real_initial_obj_pos[0] - real_final_obj_pos[0]) +
                         abs(real_initial_obj_pos[1] - real_final_obj_pos[1]) +
                         abs(real_initial_obj_pos[2] - real_final_obj_pos[2])) >= thrs
                         
                    print('real_obj_moved', real_obj_moved)
                    print('delta X', real_final_obj_pos[0] - real_initial_obj_pos[0])
                    print('delta Y', real_final_obj_pos[1] - real_initial_obj_pos[1])
    
                    ## store current delta if there was a real contact with the box
                    if real_obj_moved: 
                        ## compute real effect
                        real_obtained_effect = discr.compute_effect(real_initial_obj_pos,
                                                                    real_final_obj_pos)
                        print('real_obtained_effect', real_obtained_effect)
                        
                        if real_obtained_effect != '' and \
                            real_obtained_effect in actions_to_learn_vector:
    
                            nb_new_traj += 1
                            nb_trajs_found += 1
                            print('nb_new_traj', nb_new_traj)                            
                            
                            ## only store the traj if (init_pos, obtained_effect)
                            ## is not successful
                            obtained_traj_class = \
                                current_trajs_dict[current_init_pos, real_obtained_effect]
                            obtained_traj_res = obtained_traj_class.get_traj_res()                        
                            if obtained_traj_res != 'success':
                                print('-------------> STORE NEW (', current_init_pos, real_obtained_effect, ') FOUND!')
                                
                                for dd in extended_delta_vector:
                                    dd.set_effect(real_obtained_effect)
#                                    dd.print_me()
                                    
                                new_delta_trajs_vector += extended_delta_vector
                            
                            ## stop looking for trajs for this init_pos effect
                            actions_to_learn_vector = [effect for effect in actions_to_learn_vector
                                                                    if effect != real_obtained_effect]
    
                    ## move arm up
                    eef_pos = ros_services.call_get_eef_pose('left')
                    eef_pos = [round(pos, sim_param.round_value) for pos in eef_pos[0:3]]
                    pos_up = copy.copy(eef_pos)
                    pos_up[2] += 0.2
                    success = ros_services.call_move_to_initial_position(pos_up) 
                    if not success:
                        print("ERROR - extend dataset failed")
                    
                    ## compute new obj pos
    #                new_obj_pos = sim_param.first_obj_pos
    #                    new_obj_pos = [new_obj_pos[0] + random.uniform(-sim_param.new_obj_pos_dist,
    #                                                                   sim_param.new_obj_pos_dist),
    #                                   new_obj_pos[1] + random.uniform(-sim_param.new_obj_pos_dist,
    #                                                                   sim_param.new_obj_pos_dist),
    #                                   new_obj_pos[2]]
    #                    new_obj_pos = [round(poss, sim_param.round_value) for poss in new_obj_pos]                        
                    
                    ## move obj to init pos    
                    success = ros_services.call_restart_world("object",
                                                              sim_param.obj_name_vector[0],
                                                              initial_obj_pos)
                    if not success:
                        print("ERROR - restart_world failed for object", sim_param.obj_name_vector[0])  
    #                else:
    #                    obj_pos_dict['cube'] = new_obj_pos
                
#            nb_new_traj += 1
            counter += 1
        ## end generate traj
    
    print(nb_trajs_found, "new trajs found")
    
    return new_delta_trajs_vector, actions_to_learn_vector
    
'''
if there is NOT success for the inference of a (init_pos, effect) then
if the perf value of the (init_pos, effect) changes between two iterations its value
comes to 0, else 1 is added
'''
def update_perf_value_changes(
            perf_value_changes_vector,
            prev_iter_dataset_stats_class_perf_value_vector,
            current_iter_dataset_stats_class_perf_value_vector):
    for pos in range(len(prev_iter_dataset_stats_class_perf_value_vector)):
        if current_iter_dataset_stats_class_perf_value_vector[pos] == 'success':
            perf_value_changes_vector[pos] = -1
        else:
            if prev_iter_dataset_stats_class_perf_value_vector[pos] != \
                current_iter_dataset_stats_class_perf_value_vector[pos]:            
                perf_value_changes_vector[pos] = 0
            else:
                perf_value_changes_vector[pos] = perf_value_changes_vector[pos] + 1

    return perf_value_changes_vector    

#'''
#Remove trajs from figure
#'''
#def reset_plot(nb_initial_pos):
#    fig = plt.figure()
#    fig.set_size_inches(7, 7)
#    ax = fig.add_axes([0, 0, 1, 1])
#    
#    # set axis limits
#    plt.xlim(-1.2,1.2)
#    plt.ylim(-1.2,1.2)
##        ax.axis('off')
# 
#    # plot the origin
#    origin = Rectangle((sim_param.obj_pos[0]-sim_param.obj_side/2, 
#                        sim_param.obj_pos[1]-sim_param.obj_side/2), 
#                       sim_param.obj_side, sim_param.obj_side, fc="grey")
#    ax.add_patch(origin)
#     
#    ## plot the big circle points
#    list_x_axis, list_y_axis, list_z_axis = \
#        setup.gen_init_eef(nb_initial_pos)
#    ax.plot(list_x_axis,list_y_axis,'o',c='r')
#
#    return ax


''' 
Compute the perf value for each init pos and effect
'''
def plot_infer_trajs_iterations_effect_init_pos(iterations_dict,
                                                nb_init_pos,
                                                current_algo):
    current_iter_class = iterations_dict[nb_init_pos]
    current_random_data_stat_class = \
        (current_iter_class.get_dataset_stats_vector())[0] ## only the random dataset
    current_dataset_dict = current_random_data_stat_class.get_results_dict()       
    current_algo_class = current_dataset_dict[current_algo]
    current_algo_dict = current_algo_class.get_results_dict()
    current_dataset_size_class = current_algo_dict[sim_param.nb_init_pos_for_adaption]
    current_dataset_size_dict = current_dataset_size_class.get_indiv_traj_info()
    init_pos_results_vector = [] ## to store all the results in an iteration
    for current_init_pos in range(sim_param.nb_init_pos_for_adaption):
        init_pos_results = [] ## up, left, down, right
        for ef in ['far', 'left', 'close', 'right']:
            if not (current_init_pos, ef) in current_dataset_size_dict:
                print('Unknown effect for', (current_init_pos, ef))
                init_pos_results.append("fail")
                continue
            
            current_traj_class = \
                current_dataset_size_dict[current_init_pos, ef]
            current_effect_result = current_traj_class.get_traj_res()
            init_pos_results.append(current_effect_result)
            if current_traj_class.get_desired_effect() != \
               current_traj_class.get_obtained_effect():
                   print('Wrong effect for', 
                         current_init_pos, current_traj_class.get_desired_effect(),
                         ' instead', current_traj_class.get_obtained_effect())
        init_pos_results_vector = init_pos_results_vector + init_pos_results
    return init_pos_results_vector

'''
Test
'''

if __name__ == '__main__':
    tmp_traj = [[0.4, 0], 
                [0.3, 0.1], 
                [0.2, 0.2]]
    tmp_delta = [['orien_5','dist_0','down'], ## should be delta classes
             ['orien_4','dist_0','up'], ## vectors just to run the example
             ['orien_5','dist_0','left-down']]
    new_delta_trajs_vector = extend_trajectory(tmp_traj, 
                                               tmp_delta, 
                                               8, 
                                               7, 
                                               "down")
    for new_delta in new_delta_trajs_vector:
            new_delta.print_me()