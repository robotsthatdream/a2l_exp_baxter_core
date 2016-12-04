# -*- coding: utf-8 -*-
"""
Created on Mon Jul 11 10:21:16 2016

@author: maestre
"""
from __future__ import print_function

import matplotlib.pyplot as plt
import random
from collections import OrderedDict

class Algo_stats():
    def __init__(self, algo_name):
        self.algo_name = algo_name
        self.results = OrderedDict()
    
    def get_algo_name(self):
        return self.algo_name
    def get_results(self):
        return self.results
    def add_result(self, nb_trajs, res_values):
        self.results[nb_trajs] = res_values
    def print_me(self):
        for tmp_d_key, tmp_d_value in self.results.items():
            print(tmp_d_key, "-->", tmp_d_value) #.get_all())

class Dataset_stats():
    def __init__(self, dataset_name):
        self.dataset_name = dataset_name ## example extended
        self.algo_results = OrderedDict()
    
    def get_dataset_name(self):
        return self.dataset_name
    def get_results(self):
        return self.algo_results
    def add_algo_results(self, current_algo_name, current_algo_results):
        self.algo_results[current_algo_name] = current_algo_results

    
''' 
Print stats 
'''
def plot_global_results_dataset_size(extended_stats):
    dataset_name = extended_stats.get_dataset_name()
    algo_res_dict = extended_stats.get_results()
    extended_algo_name_vector = algo_res_dict.keys()
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    fig.set_size_inches(8, 10)
    fig.text(0.9, 0.95, dataset_name.upper() + ' DATASET', 
             ha='right', va='center', style='italic')
    fig.text(0.5, 0.09, 'Number of initial arm positions', 
             ha='center', va='center', fontsize=13)
    fig.text(0.06, 0.5, 'Result of inferred trajectories (%)', 
             ha='center', va='center', rotation='vertical', fontsize=13)       
    
    for current_algo in extended_algo_name_vector:
    
        algo_results = algo_res_dict[current_algo].get_results()
        keys = algo_results.keys()
        traj_res = algo_results.values()
        nb_trajs = sum(traj_res)/len(keys)
        for res in traj_res:
            for pos in range(len(res)):
                res[pos] = (float(res[pos])/nb_trajs)*100
        
        plt.rcParams.update({'font.size': 11})
    
        if current_algo == 'hand-coded':
            _tmp = 'Hand-coded'
            _ax = ax1
            
        elif current_algo == 'hillclimbing':
            _tmp = 'Hillclimbing'
            _ax = ax2
            
        elif current_algo == 'k2':
            _tmp = 'K2'
            _ax = ax3
            
        else:
            _tmp = ''  
        
        _ax.set_title(_tmp.upper())# + ' - ' + dataset_name.upper() + ' DATASET')    
        _ax.set_ylim(0, 100)    
        _ax.tick_params(axis='both', which='major', labelsize=12)
        _ax.tick_params(axis='both', which='minor', labelsize=12)
        
        pos = 0
        while pos < 3:
            if pos == 0:
                traj_type = 'Success'
                color = 'blue'
                linestyle='solid'
            elif pos == 1:
                traj_type = 'False positive'
                color = 'green'
                linestyle='dashed'
            elif pos == 2:
                traj_type = 'Failure'
                color = 'red'
                linestyle='dotted'
            else:
                print('ERROR stats')
            current_traj_type_values = [v[pos] for v in traj_res]
            _ax.plot(keys, current_traj_type_values, 
                    '-*', 
                    color = color,
                    markersize=5, linewidth=1,
                    label = traj_type,
                    linestyle = linestyle
                    )
            
            pos += 1
    
    ax1.legend(('Success', 'False positive', 'Failure'), 
               fancybox=True,
               loc='upper left',
               fontsize='small')
    for _ax in [ax1, ax2, ax3]:
        _ax.set_xticks(keys)
    
    plt.show()

'''
Main
'''

## Fake stats
extended_stats = Dataset_stats('extended')

algo_vector = ['hand-coded', 'hillclimbing', 'k2']
for current_algo in algo_vector:
    current_algo_stats = Algo_stats(current_algo)
    
    for i in range(5):
        success_value = random.randint(0, 5)
        f_p_value = random.randint(0, 5)
        if success_value + f_p_value > 10:
            f_p_value = 10 - success_value
            if not f_p_value == 0:
                f_p_value -= 1
            fail_value = 1
        else:
            fail_value = 10- success_value - f_p_value       
        res_stats = [success_value, f_p_value, fail_value]
        current_algo_stats.add_result(4*i, res_stats)
        
    current_algo_stats.print_me()
    extended_stats.add_algo_results(current_algo, current_algo_stats)

## call plot function
plot_global_results_dataset_size(extended_stats)