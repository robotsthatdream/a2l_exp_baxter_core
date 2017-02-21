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
import inference_ros as infer_ros
import learning as ln
import simulation_parameters as sim_param
import inf_traj_series_dataset_classes as dataset_series_classes
if sim_param.print_time:
        import time

def run_exp_dataset_size(dataset_type_vector,
                        learn_algo_vector,
                        new_obj_pos,
                        output_folder, 
                        current_results_folder, 
                        current_plot_folder):
                            
    dataset_stats_vector = [] ## to store the stats of each dataset
    for dataset_type in dataset_type_vector:
        print('\n\n----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('Generating and discretizing', dataset_type.upper(), 'dataset')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------')
        print('----------------------------------------------------------\n\n')        
        if dataset_type == 'directed':
            filename = sim_param.generated_files_folder + 'directed_discr_wps.csv'
        elif dataset_type == 'extended':
            filename = sim_param.generated_files_folder + 'extended_discr_wps.csv'
        elif dataset_type == 'random':
            filename = sim_param.generated_files_folder + 'random_discr_wps.csv'        
        else:
            print('ERROR - main - wrong dataset_type value')   

        ## Create the classes to store the different stats results             
        current_dataset_stats = dataset_series_classes.Dataset_stats(dataset_type)
        for tmp_algo in learn_algo_vector:
            tmp_algo_stats = dataset_series_classes.Algo_stats(tmp_algo)
            current_dataset_stats.add_algo_results(tmp_algo, 
                                                   tmp_algo_stats)                              
    
        if sim_param.print_time:
            t = time.process_time()
        ## create raw dataset
        raw_delta_vector = dataset.create_dataset(dataset_type,
                                                  sim_param.nb_min_init_pos)

        orientation_discr_vector = \
            [sim_param.nb_min_orientation_sections * x for x in 
            range(1, sim_param.orientation_discr_vector_size+1)]
                
        distance_discr_vector = \
            [sim_param.nb_min_distance_sections + x for x 
            in range(0, sim_param.distance_discr_vector_size)]

        for current_orien in orientation_discr_vector:
            for current_dist in distance_discr_vector:
                print('\n\n----------------------------------------------------------')
                print('----------------------------------------------------------')
                print('ORIENTATION VALUE', current_orien)
                print('DISCRETIZATION VALUE', current_dist)
                print('----------------------------------------------------------')
                print('----------------------------------------------------------\n\n')   
                                                  
                ## create discretized dataset
                discr_delta_vector = discr.discretize_trajs(raw_delta_vector,
                                                              current_orien,
                                                              current_dist)
                if dataset_type == 'random': ## remove the very small deltas
                    nb_removed = 0
                    for discr_delta in discr_delta_vector:
                        if discr_delta[2] == 'zero':
                            discr_delta_vector.remove(discr_delta)
                            nb_removed += 1
                    print(nb_removed,'very small movements were removed')                        
                
                ## store discretized dataset
                discr.save_discr_deltas(filename, discr_delta_vector)
                if sim_param.print_time:
                    print('Elapsed time for create_dataset', 
                          time.process_time() - t)
                
                ## plot discretized dataset stats
                if sim_param.plot_dataset_stats:
                    discr.plot_dataset_stats(filename)                
                    
                for current_algo in learn_algo_vector:            
                    print('\n\n----------------------------------------------------------')
                    print('----------------------------------------------------------')
                    print('Learning BN in the', dataset_type.upper(), 'dataset',
                          'with', current_algo.upper(),  'algorithm')
                    print('----------------------------------------------------------')
                    print('----------------------------------------------------------\n\n')                   
                    
                    ## Learn BN
                    if sim_param.print_time:
                        t = time.process_time()
                    bn = ln.learn_bn(filename, current_algo)
                    bn_url = 'BN_' + dataset_type + '_' + current_algo + '.bif'                
                    ln.save_bn(bn, 
                               sim_param.generated_files_folder + bn_url)                           
                    if sim_param.print_time:
                        print('Elapsed time for learn_bn', 
                              time.process_time() - t)
            
                    ## Infere trajs
                    if sim_param.print_time:
                        t = time.process_time()            
                    current_stats = infer_ros.infere_trajectories(current_results_folder,
                                                              bn, 
                                                              current_algo,
                                                              dataset_type,
                                                              new_obj_pos,
                                                              sim_param.nb_min_init_pos,
                                                              current_orien,
                                                              current_dist)
                    if sim_param.print_time:
                        print('Elapsed time for infere_trajs', 
                              time.process_time() - t)
                    
                    ## Store stats
                    current_algo_stats_dict = \
                        current_dataset_stats.get_algo_results(current_algo)
                    current_algo_stats_dict.add_result((current_orien, current_dist),
                                                       current_stats)
                    if sim_param.debug:
                        current_algo_stats_dict.print_me()
            
        dataset_stats_vector.append(current_dataset_stats)
        
    return dataset_stats_vector, \
            orientation_discr_vector, \
            distance_discr_vector
