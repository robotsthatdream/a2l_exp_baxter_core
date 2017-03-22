# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

import os, sys
lib_a2l_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'a2l_core_lib')))
sys.path.append(lib_a2l_path)
lib_exp_path = os.path.realpath(os.path.abspath(os.path.join('.', 'lib', 'experiment_lib')))
sys.path.append(lib_exp_path)
import dataset_generation as dataset
import discretize as discr
import validation as validation
import learning as ln
import simulation_parameters as sim_param
import inf_traj_series_dataset_classes as dataset_series_classes
if sim_param.print_time:
        import time        
import ros_services

'''
Run basic learning
'''
def basic_learning_dataset_size(dataset_type_vector,
                                learn_algo_vector,
                                current_results_folder,
                                current_generated_files_folder,
                                current_plot_folder,
                                left_gripper_interface):
          
    ## Create discretizations
    current_orien_discr = discr.compute_orientation_discr()
    current_inclin_discr = discr.compute_inclination_discr()
    current_dist_discr = discr.compute_distance_discr()
    
    dataset_stats_vector = []     
    current_nb_initial_pos = sim_param.nb_min_init_pos   
    for dataset_type in dataset_type_vector:
        print('\n\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('Generating and discretizing', dataset_type.upper(), 'dataset')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')    
             
        ## Raw dataset name
        if sim_param.real_robot:
            raw_dataset_filename = \
                '/home/maestre/.ros/eef_trajectory_recorder.csv'                        
        else:
            if dataset_type == 'directed':
                raw_dataset_filename = sim_param.generated_datasets_folder + 'directed_dataset.csv'  
            elif dataset_type == 'random':
                raw_dataset_filename = sim_param.generated_datasets_folder + 'random_dataset.csv'
            else:
                print('ERROR - main - wrong dataset_type value')                 

        ## Create raw dataset
        raw_delta_vector = []
        if sim_param.new_dataset:
            if dataset_type == 'directed':
                ros_services.call_generate_directed_dataset(dataset_type)
            elif dataset_type == 'random':
                dataset.create_dataset(dataset_type, 
                                       current_nb_initial_pos)
            else:
                print('ERROR - main - wrong dataset_type value')
   
        ## Create the classes to store the different stats results 
        ## Dataset            
        current_dataset_stats = dataset_series_classes.Dataset_stats(dataset_type)
        ## Algo
        for tmp_algo in learn_algo_vector:
            tmp_algo_stats = dataset_series_classes.Algo_stats(tmp_algo)
            current_dataset_stats.add_algo_results(tmp_algo, 
                                                   tmp_algo_stats)                                                   
        basic_size = sim_param.nb_min_init_pos       
        nb_initial_pos_vector = [basic_size * repetitions 
                                for repetitions in 
                                range(1, sim_param.nb_dataset_sizes+1)]                                                      
        
        ## Tranform raw delta dataset into deltas (only directed)
        if dataset_type == 'directed':
            raw_delta_vector = dataset.read_dataset(raw_dataset_filename)
            
        ## Read available raw delta dataset (only random)
        elif dataset_type == 'random':
#            raw_deltas_dataset_filename = sim_param.generated_datasets_folder + 'random_dataset_deltas.csv'
#            raw_delta_vector = dataset.read_delta_dataset(raw_deltas_dataset_filename)
            raw_delta_vector = dataset.read_dataset(raw_dataset_filename)
        else:
            print('ERROR - main - wrong dataset_type value')
        
        ## discretize dataset            
        discr_delta_vector = discr.discretize_trajs(
                                    raw_delta_vector,
                                    current_orien_discr,
                                    current_inclin_discr,
                                    current_dist_discr)
                                
        if dataset_type == 'random': ## remove the very small deltas
            nb_removed = 0
            for discr_delta in discr_delta_vector:
                if discr_delta[1] == 'zero':
                    discr_delta_vector.remove(discr_delta)
                    nb_removed += 1
            if nb_removed > 0:
                print(nb_removed,'very small movements were removed')                        
        
        ## store discretized dataset
        if dataset_type == 'directed':
            discr_dataset_filename = current_generated_files_folder + 'directed_discr_wps_initial.csv'
        elif dataset_type == 'random':
            discr_dataset_filename = current_generated_files_folder + 'random_discr_wps_initial.csv'
        else:
            print('ERROR - main - wrong dataset_type value')        
        discr.save_discr_deltas(discr_dataset_filename, discr_delta_vector)
        
        ## plot hard-coded discretized dataset stats
        if sim_param.plot_dataset_stats and sim_param.discr_hand_coded:
            discr.plot_dataset_stats(discr_dataset_filename)
            
        for current_algo in learn_algo_vector:            
            print('\n\n----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('Learning BN in the', dataset_type.upper(), 'dataset',
                  'with', current_algo.upper(),  'algorithm')
            print('INITIAL BABBLING')                    
            print('----------------------------------------------------------')
            print('----------------------------------------------------------\n\n')                   

            dataset_size_class = \
                dataset_series_classes.Dataset_size_results(current_nb_initial_pos)
                                                   
            ## Learn BN
            if sim_param.print_time:
                t = time.process_time()
            bn = ln.learn_bn(discr_dataset_filename, current_algo)
            bn_url = 'BN_' + dataset_type + '_initial_' + current_algo  + '.bif'                
            ln.save_bn(bn, 
                       current_generated_files_folder + bn_url)                           
            if sim_param.print_time:
                print('Elapsed time for learn_bn', 
                      time.process_time() - t)

            curr_nb_infere_trajs = 0
            if sim_param.experiment_type == 'a2l_reproduce_dataset':
                max_nb_infere_trajs = sim_param.nb_infere_trajs
            else:
                max_nb_infere_trajs = 1
                
            print('\n----------------------------------------------------------')
            print('----------------------------------------------------------')
            print('Validating BN ')
            print('INITIAL BABBLING')  
            print('----------------------------------------------------------')
            print('----------------------------------------------------------\n\n')                  
                
            while curr_nb_infere_trajs < max_nb_infere_trajs:                                    
                ## validate affordance knowledge                                
                validation.affordance_validation(
                    current_results_folder,
                    bn, 
                    current_algo,
                    dataset_size_class,
                    dataset_type,         
                    current_nb_initial_pos,
                    curr_nb_infere_trajs,
                    current_orien_discr,
                    current_inclin_discr,
                    current_dist_discr,
                    left_gripper_interface)
                curr_nb_infere_trajs += 1
                                           
            ## Store stats and move's mean prob                 
            current_algo_stats_class = \
                current_dataset_stats.get_algo_results(current_algo)                
            current_algo_stats_class.add_result(current_nb_initial_pos,
                                                dataset_size_class)
                                               
#            if sim_param.debug:
#                current_algo_stats_class.print_me()                                        
        dataset_stats_vector.append(current_dataset_stats)

    return dataset_stats_vector, \
            nb_initial_pos_vector, \
            current_orien_discr,\
            current_inclin_discr,\
            current_dist_discr, \
            raw_delta_vector, \
            discr_dataset_filename
'''
Run adaptive learning
'''
def adaptive_learning_dataset_size(current_dataset_stats, ## random
                                    current_nb_initial_pos, ## 8
                                    learn_algo_vector,
                                    current_results_folder,
                                    current_generated_files_folder,
                                    current_orien_discr,
                                    current_inclin_discr,
                                    current_dist_discr,
                                    current_iteration,
                                    previous_raw_delta_vector,
                                    perf_value_changes_vector,
                                    prev_inferred_discr_delta_vector,
                                    left_gripper_interface):
    
    ''' Extend dataset '''    
    dataset_stats_vector = [] 
    current_dataset_name = current_dataset_stats.get_dataset_name()
    ## Create the classes to store the different stats results 
    ## Dataset            
    extended_dataset_stats = dataset_series_classes.Dataset_stats('random')
    ## Algo
    for tmp_algo in learn_algo_vector:
        tmp_algo_stats = dataset_series_classes.Algo_stats(tmp_algo)
        extended_dataset_stats.add_algo_results(tmp_algo, 
                                               tmp_algo_stats)

    print('\n\n----------------------------------------------------------')
    print('----------------------------------------------------------')
    print('Extending', current_dataset_name.upper(), 'dataset')
    print('ITERATION', current_iteration)    
    print('----------------------------------------------------------\n\n')   
    
    ## For each successful trajectory generate new ones
    print("----->Extending trajectory".upper())
    
    new_raw_delta_vector, effect_directed_extension_vector = \
        dataset.extend_dataset(
            current_dataset_stats,
            sim_param.nb_init_pos_for_adaption,
            learn_algo_vector,
            perf_value_changes_vector,
            len(previous_raw_delta_vector),
            left_gripper_interface)
    
    extended_raw_delta_vector = previous_raw_delta_vector + \
                                new_raw_delta_vector
    print('Num RAW deltas before extension:', len(previous_raw_delta_vector))
    print('Num RAW deltas after extension:', len(extended_raw_delta_vector))
    
    ## discretize dataset
    
    extended_discr_delta_vector = discr.discretize_trajs(
                                extended_raw_delta_vector,
                                current_orien_discr,
                                current_inclin_discr,
                                current_dist_discr)
    print('Num DISCRETE deltas after extension:', len(extended_discr_delta_vector)) 
#    extended_discr_delta_vector = extended_discr_delta_vector ## + prev_inferred_extended_discr_delta_vector
    
    ## remove small moves
    nb_removed = 0
    for discr_delta in extended_discr_delta_vector:
        if discr_delta[3] == 'zero':
            extended_discr_delta_vector.remove(discr_delta)
            nb_removed += 1
    if nb_removed > 0:
        print(nb_removed,'very small movements were removed')
        print('Num DISCRETE deltas after extension after ZEROS:', len(extended_discr_delta_vector))                       
    
    ## store discretized dataset
    discr_dataset_filename = current_generated_files_folder + \
                'random_discr_wps_iteration_' + str(current_iteration) + '.csv'
    discr.save_discr_deltas(discr_dataset_filename, 
                            extended_discr_delta_vector)                
    
    inferred_discr_delta_vector = []
    for current_algo in learn_algo_vector:            
        print('\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('Learning BN in the EXTENDED', current_dataset_name.upper(), 'dataset',
              'with', current_algo.upper(),  'algorithm')
        print('ITERATION', current_iteration)                  
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')                   
                                               
        ## Learn BN
        bn = ln.learn_bn(discr_dataset_filename, current_algo)
        bn_url = 'BN_' + current_dataset_name + '_' + str(current_iteration)  + '_' + current_algo + '.bif'                
        ln.save_bn(bn, 
                   current_generated_files_folder + bn_url)                           

        print('\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('Validating BN ')
        print('ITERATION', current_iteration)    
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')  
     
        ''' Evaluation '''
        dataset_size_class = \
            dataset_series_classes.Dataset_size_results(current_nb_initial_pos)        
        tmp_inferred_discr_delta_vector = \
            validation.affordance_validation(
                current_results_folder,
                bn, 
                current_algo,
                dataset_size_class,
                current_dataset_name,          
                current_nb_initial_pos,
                current_iteration,
                current_orien_discr,
                current_inclin_discr,
                current_dist_discr,
                left_gripper_interface)
        if len(tmp_inferred_discr_delta_vector) > 0:
            print(current_algo, len(tmp_inferred_discr_delta_vector))
            inferred_discr_delta_vector = inferred_discr_delta_vector +\
                                          tmp_inferred_discr_delta_vector        
                                                                         
        ## Store stats and move's mean prob
        extended_algo_stats_class = \
            extended_dataset_stats.get_algo_results(current_algo)                
        extended_algo_stats_class.add_result(current_nb_initial_pos,
                                             dataset_size_class)
                                           
        if sim_param.debug:
            extended_algo_stats_class.print_me()
    dataset_stats_vector.append(extended_dataset_stats)
            
    ''' Return stats, extended dataset and vector of extended (initpos, effect)'''
    return dataset_stats_vector, \
            extended_raw_delta_vector, \
            effect_directed_extension_vector, \
            inferred_discr_delta_vector

            