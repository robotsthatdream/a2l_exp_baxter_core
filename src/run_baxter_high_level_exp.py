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
import copy


'''
a
'''
def compute_max_effect(mean_prob_vector):
    if len(mean_prob_vector) == 0:
        return '', []

    first_effect_info = mean_prob_vector[0]
    max_arm = first_effect_info[0]    
    max_effect = first_effect_info[1]
    max_prob = first_effect_info[2]
    max_traj = first_effect_info[3]    
    
    for curr_effect_info in mean_prob_vector[1:]:
        if curr_effect_info[2] > max_prob:
            max_arm = curr_effect_info[0]
            max_effect = curr_effect_info[1]
            max_prob = curr_effect_info[2]
            max_traj = curr_effect_info[3]            
            
    if max_prob == 0: ## no trajs
        max_effect = ''
        
    return max_effect, max_traj, max_arm
    
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
        
#    bn_folder = '/home/maestre/git/a2l_exp_baxter_core/src/generated_files/dataset_size/2017-03-25_20:05:50_16_3'
#    iter_nb = 3
#    bn_full_path = bn_folder + '/BN_random_' + str(iter_nb) + '_' + learn_algo + '.bif'
    bn_full_path = '/home/maestre/Desktop/BN_video1_predefined_hc.bif'

    ''' Load the BN '''
    bn_loaded = agrum.loadBN(bn_full_path)
    
    ''' Create discretizations '''
    current_orien_discr = discr.compute_orientation_discr()
    current_inclin_discr = discr.compute_inclination_discr()
    current_dist_discr = discr.compute_distance_discr()    
    
#    ''' Load eef interface to close gripper'''
#    rospy.init_node('left_gripper_node', anonymous=True)
#    left_gripper_interface = baxter_interface.Gripper('left')
#    rospy.init_node('right_gripper_node', anonymous=True)
#    right_gripper_interface = baxter_interface.Gripper('right')
#    rospy.sleep(1)
 
 
    if not sim_param.real_robot:
        ''' Restart scenario '''
        success = ros_services.call_restart_world("all")
        if not success:
            print("ERROR - restart_world failed")  
            
    else:
        ''' Right arm to init pos '''
    res_init_pos = ros_services.call_move_to_initial_position([0.65, 0.3, -0.145],
                                                              'left')
        
    ''' Right arm to init pos '''
    res_init_pos = ros_services.call_move_to_initial_position([0.65, -0.1, -0.145],
                                                              'right')
            
    ''' To infer next move '''
    ie = agrum.LazyPropagation(bn_loaded)         
    
#    orig_eef_pos = ros_services.call_get_eef_pose('left')
#    orig_eef_pos = [round(pos, sim_param.round_value) for pos in orig_eef_pos[0:3]]    

    arm_pos_dict = {'left':[], 'right':[]}
    chosen_arm = 'left'
    other_arm= 'right'
    nb_iter = 0
    for nb_iter in range(25):

        print('\n\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('Iteration', nb_iter)
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')           
#        tmp_arm_vector = arm_pos_dict.keys() ## to respect arm order when moving back to init pos
        
        ''' Each eef to init pos'''
#        for curr_arm in [tmp_arm_vector[0], tmp_arm_vector[1]]:
        for curr_arm in [chosen_arm, other_arm]:
            
#            ## move arm up
#            eef_pos = ros_services.call_get_eef_pose(curr_arm)
#            eef_pos = [round(poss, sim_param.round_value) for poss in eef_pos[0:3]]
#            pos_up = copy.copy(eef_pos)
#            pos_up[2] += 0.2
#            success = ros_services.call_move_to_initial_position(pos_up,
#                                                                 curr_arm) 
#            if not success:
#                print("ERROR - extend dataset failed")            

            print(curr_arm)
        
            ''' Set eef beside box '''
            curr_obj_pos = ros_services.call_get_model_state(obj_name)
            curr_obj_pos = [v for v in curr_obj_pos[0:3]]
            curr_obj_pos[2] = -0.145
            curr_eef_pos = copy.copy(curr_obj_pos)
            if curr_arm == 'left':
                curr_eef_pos[1] += sim_param.radio
            else:
                curr_eef_pos[1] -= sim_param.radio
            
            ''' Modify eef pos '''            
            tmp = random.uniform(-sim_param.radio/4, sim_param.radio/4)
            curr_eef_pos[0] = curr_obj_pos[0] + tmp
            if curr_obj_pos[1] < curr_eef_pos[1]: ## eef to the left of the box
                tmp = random.uniform(sim_param.radio/2,sim_param.radio*1.5)
                curr_eef_pos[1] = curr_obj_pos[1] + tmp
            else:
                tmp = random.uniform(-sim_param.radio/2,-sim_param.radio*1.5)
                curr_eef_pos[1] = curr_obj_pos[1] + tmp
                
            print(curr_eef_pos)
    
            res_init_pos = ros_services.call_move_to_initial_position(curr_eef_pos,
                                                                      curr_arm)
            arm_pos_dict[curr_arm] = curr_eef_pos
                                                                
            ## TODO check result        
            print("curr_eef_pos", curr_arm, curr_eef_pos)                
        
        ''' Compute highest mean prob for each effect from each arm '''
        mean_prob_vector = []
        for curr_arm in [chosen_arm, other_arm]:

            curr_eef_pos = arm_pos_dict[curr_arm]
                        
            for possible_effect in sim_param.effect_values:
                print('\n----------------------------------------------------------')
                print('Possible effect', curr_arm, possible_effect)            
                tmp_traj, sim_obj_moved, mean_traj_prob = \
                    run.simulate_traj(bn_loaded, ie, 
                                       curr_eef_pos,
                                       {'cube':curr_obj_pos},
                                       possible_effect, 
                                       current_orien_discr,
                                       current_inclin_discr,
                                       current_dist_discr)            
                if sim_obj_moved:
                    mean_prob_vector.append([curr_arm,
                                             possible_effect,
                                             mean_traj_prob,
                                             tmp_traj])
                else:
                    mean_prob_vector.append([curr_arm,
                                             possible_effect,
                                             0,
                                             tmp_traj])                                              
        for v in mean_prob_vector:
            print(v[0], 
                  v[1], 
                  v[2])                    
    
        ''' Select effect with highest mean prob to run '''
        expected_effect, best_traj, chosen_arm = \
            compute_max_effect(mean_prob_vector)
            
        if expected_effect == '':
            print("\nNo trajectory found")
        else:
            print("\nSelected arm:", chosen_arm) 
            print("Expected_effect", expected_effect) 
                        
            ''' Non selected arm to safe pos '''
            if chosen_arm == 'left':
                other_arm = 'right'
                chosen_eef_pos = arm_pos_dict[chosen_arm]
                other_arm_pos = arm_pos_dict[other_arm]
                other_arm_pos[1] -= 0.2
            else:
                other_arm = 'left'
                chosen_eef_pos = arm_pos_dict[chosen_arm]
                other_arm_pos = arm_pos_dict[other_arm]
                other_arm_pos[1] += 0.2
                
            ros_services.call_move_to_initial_position(other_arm_pos,
                                                       other_arm)
            tmp_arm_vector = [] 
            tmp_arm_vector.append(chosen_arm)
            tmp_arm_vector.append(other_arm)
            print('tmp_arm_vector', tmp_arm_vector)
            
            ''' Plot traj with highest prob '''
            run.plot_save_traj_3d([chosen_eef_pos], ## to plot eef pos
                                  best_traj,
                                  {'cube':curr_obj_pos},
                                  chosen_eef_pos,
                                  expected_effect,
                                  "", ## not save traj
                                  True) ## sim_obj_moved
            
    #        ''' Check if traj under box '''
    #        below_box = False
    #        pos = 0
    #        while not below_box and pos < len(traj):
    #            below_box = traj[pos] < -0.17
    #            pos += 1  
    #        if sim_param.debug_infer:
    #            print('\nbelow_box:', below_box)        
        
            ''' Execute trajectory, listening to feedback '''
            tc = real_exec.Traj_callback(
                    best_traj,
                    True, ## sim_obj_moved
                    current_orien_discr, current_inclin_discr, current_dist_discr,
                    False, ##below_box
                    chosen_arm)
            
            if not sim_param.real_robot:
            
                ''' Reset box position if needed '''
                curr_obj_pos = ros_services.call_get_model_state(obj_name)
                if sim_param.debug_infer:
                    print('euclidean dist:', distance.euclidean(curr_obj_pos, sim_param.first_obj_pos), \
                                            '<', sim_param.obj_too_far_distance)                     
                if distance.euclidean(curr_obj_pos, 
                               sim_param.first_obj_pos) > sim_param.obj_too_far_distance*2:        
                    print('-------------> UPDATING CUBE POSITION!')
        
                    ## move arm up
                    for curr_arm in tmp_arm_vector:
                        eef_pos = ros_services.call_get_eef_pose(curr_arm)
                        eef_pos = [round(poss, sim_param.round_value) for poss in eef_pos[0:3]]
                        if eef_pos[2] < 0:
                            pos_up = copy.copy(eef_pos)
                            pos_up[2] += 0.2
                            success = ros_services.call_move_to_initial_position(pos_up,
                                                                                 curr_arm) 
                            if not success:
                                print("ERROR - extend dataset failed")
                    
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