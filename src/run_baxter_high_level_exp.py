#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

from scipy.spatial import distance
import random

import pyAgrum as agrum

import os, sys
lib_a2l_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'a2l_core_lib')))
sys.path.append(lib_a2l_path)
lib_exp_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'experiment_lib')))
sys.path.append(lib_exp_path)
#import dataset_generation as dataset
#import dataset_stats as dataset_stats
#import statistics as stats
import simulation_parameters as sim_param
#import experiment_a2l as exp_a2l
#import experiment_discretization as exp_discr
#import inference_ros as infer_ros
#import iteration_classes as iter_class
import validation as run
import discretize as discr
import traj_callback as real_exec
import ros_services

import rospy
import baxter_interface 


'''
a
'''
def compute_max_effect(mean_prob_vector):
    if len(mean_prob_vector) == 0:
        return '', []

    first_effect_info = mean_prob_vector[0]
    max_effect = first_effect_info[0]
    max_prob = first_effect_info[1]
    max_traj = first_effect_info[2]
    
    for curr_effect_info in mean_prob_vector[1:]:
        if curr_effect_info[1] == max_prob:
            max_effect = curr_effect_info[0]
            max_prob = curr_effect_info[1]
            max_traj = curr_effect_info[2]
        
    return max_effect, max_traj
    
'''
Main
'''
if __name__ == "__main__":
    
    print('\n----------------------------------')
    print('----------------------------------')
    print('----------------- HIGH LEVEL EXPERIMENT - PLAY WITH BOX')
 
    ''' Experiment configuration '''
    sim_param.save_trajs = False
    
    obj_name = 'cube'
    
#    learn_algo = 'hillclimbing'
    learn_algo = 'hard-coded'
        
    bn_folder = '/home/maestre/git/a2l_exp_baxter_core/src/generated_files/dataset_size/2017-03-25_20:05:50_16_3'
    iter_nb = 3
    bn_full_path = bn_folder + '/BN_random_' + str(iter_nb) + '_' + learn_algo + '.bif'

    ''' Load the BN '''
    bn_loaded = agrum.loadBN(bn_full_path)
    
    ''' Create discretizations '''
    current_orien_discr = discr.compute_orientation_discr()
    current_inclin_discr = discr.compute_inclination_discr()
    current_dist_discr = discr.compute_distance_discr()    
    
    ''' Load eef interface to close gripper'''
#    rospy.init_node('left_gripper_node', anonymous=True)
#    left_gripper_interface = baxter_interface.Gripper('left')
#    rospy.sleep(1)
 
    ''' Restart scenario '''
    success = ros_services.call_restart_world("all")
    if not success:
        print("ERROR - restart_world failed")    
        
    ''' To infer next move '''
    ie = agrum.LazyPropagation(bn_loaded)         

    nb_iter = 0
    for nb_iter in range(50):
        
        ''' Set eef around object '''
        curr_eef_pos = ros_services.call_get_eef_pose('left')
        curr_eef_pos = [round(pos, sim_param.round_value) for pos in curr_eef_pos[0:3]]
        
        curr_obj_pos = ros_services.call_get_model_state(obj_name)
        eef_pos = [0,0,0]
        eef_pos[0] = curr_obj_pos[0] + random.uniform(-sim_param.new_obj_pos_dist,
                                                       sim_param.new_obj_pos_dist)
        if curr_obj_pos[1] > curr_eef_pos[1]:
            curr_eef_pos[1] = curr_eef_pos[1] + random.uniform(0,sim_param.new_obj_pos_dist)
        elif curr_obj_pos[1] < curr_eef_pos[1]:
            curr_eef_pos[1] = curr_eef_pos[1] + random.uniform(-sim_param.new_obj_pos_dist/2,0)
        
        eef_pos[2] = sim_param.eef_z_value

        res_init_pos = ros_services.call_move_to_initial_position(eef_pos)        
        ## TODO check result        
        print("eef_pos", eef_pos)                
        
        ''' Compute highest mean prob for each effect '''
        mean_prob_vector = []
        for possible_effect in sim_param.effect_values:
            traj, sim_obj_moved, mean_traj_prob = \
                run.simulate_traj(bn_loaded, ie, 
                                   curr_eef_pos,
                                   {'cube':curr_obj_pos},
                                   possible_effect, 
                                   current_orien_discr,
                                   current_inclin_discr,
                                   current_dist_discr)            
            
            mean_prob_vector.append([possible_effect,
                                      mean_traj_prob,
                                      traj])

        ''' Select effect with highest mean prob to run '''
        expected_effect, traj = compute_max_effect(mean_prob_vector)
        
        ''' Plot traj with highest prob '''
        run.plot_save_traj_3d([curr_eef_pos], ## to plot eef pos
                              traj,
                              {'cube':curr_obj_pos},
                              curr_eef_pos,
                              expected_effect,
                              "", ## not save traj
                              True) ## sim_obj_moved
        
        ''' Check if traj under box '''
        below_box = False
        pos = 0
        while not below_box and pos < len(traj):
            below_box = traj[pos] < -0.17
            pos += 1  
        if sim_param.debug_infer:
            print('\nbelow_box:', below_box)        
    
        ''' Execute trajectory, listening to feedback '''
        tc = real_exec.Traj_callback(
                traj,
                True, ## sim_obj_moved
                current_orien_discr, current_inclin_discr, current_dist_discr,
                below_box)
                
        print('Box pushed:', tc.obtained_effect)
        
        ''' Reset box position if needed '''
        curr_obj_pos = ros_services.call_get_model_state(obj_name)
        if sim_param.debug_infer:
            print('euclidean dist:', distance.euclidean(curr_obj_pos, sim_param.first_obj_pos), \
                                    '<', sim_param.obj_too_far_distance)                     
        if distance.euclidean(curr_obj_pos, 
                       sim_param.first_obj_pos) > sim_param.obj_too_far_distance:        
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
                                           
         
    print('\nDone.')