#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

from time import gmtime, strftime
from collections import OrderedDict
import numpy as np

import os, sys
lib_a2l_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'a2l_core_lib')))
sys.path.append(lib_a2l_path)
lib_exp_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'experiment_lib')))
sys.path.append(lib_exp_path)
import dataset_generation as dataset
import dataset_stats as dataset_stats
import statistics as stats
import simulation_parameters as sim_param
import experiment_a2l as exp_a2l
import experiment_discretization as exp_discr
import inference_ros as infer_ros
import iteration_classes as iter_class
import ros_services

import rospy
import baxter_interface 
    
'''
Main
'''
if __name__ == "__main__":
    
    print('\n----------------------------------')
    print('----------------------------------')
    print('----------------- HIGH LEVEL EXPERIMENT - PLAY WITH BOX')
 
    ''' Experiment configuration '''
#    learn_algo_vector = ['hard-coded', 'hillclimbing', 'k2']    
#    learn_algo_vector = ['hillclimbing']
    learn_algo_vector = ['hard-coded']
#    learn_algo_vector = ['hard-coded', 'hillclimbing']    
    
    ''' Load eef interface to close gripper'''
    rospy.init_node('left_gripper_node', anonymous=True)
    left_gripper_interface = baxter_interface.Gripper('left')
#    rospy.sleep(1)
 
    ''' Restart scenario '''
    success = ros_services.call_restart_world("object")
    if not success:
        print("ERROR - restart_world failed")    

    max_iterations = 50
    nb_iter = 0
    for nb_iter in range(max_iterations):
        
        ''' Set eef around object '''
        
        
        ''' Compute higher mean prob for each effect '''
        
        
        ''' Run action '''

        
        ''' Reset box if needed '''
                                           
         
    print('\nDone.')