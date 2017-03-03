# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 16:54:57 2016

@author: maestre
"""
from __future__ import print_function

import scipy.spatial.distance as spatial
import numpy as np
import collections
import random
from math import sqrt

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

class Sections:

    def __init__(self, min_value, max_value, nb_sections):
        self.section_name = 'dist_'
        self.min_value = min_value
        self.max_value = max_value
        
        ## create sections
        self.raw_sections = np.linspace(min_value, max_value, nb_sections + 1)
        ## if some need to be removed
        if sim_param.discr_random:
            nb_random_values = len(self.raw_sections)//nb_sections
            for v in range(nb_random_values):
                to_remove = random.randint(1,len(self.raw_sections)-2)
                self.raw_sections = np.delete(self.raw_sections, to_remove)        
        self.raw_sections = [round (v, sim_param.round_value) 
                            for v in self.raw_sections]
        ## label sections
        self.sections = collections.OrderedDict()
        pos = 0
        for value in self.raw_sections[:-1]:
            self.sections[self.section_name+str(pos)] = \
                [value, self.raw_sections[pos+1]]
            pos += 1        

    ''' Get the raw sections (list) '''
    def get_raw_sections(self):
        return self.raw_sections
        
    ''' Get the sections (dictionary) '''
    def get_sections(self):
        return self.sections        
        
    ''' Print the discretized sections'''        
    def print_me(self):        
        for tmp_d_key, tmp_d_value in self.sections.items():
            print(tmp_d_key, "-->", tmp_d_value)

    ''' Compute the section of a given pos '''
    def compute_section(self,eef_pos, current_obj_pos):
        current_dist = 0
        for pos in range(len(eef_pos)):
            current_dist += abs((current_obj_pos[pos] - eef_pos[pos])**2)
        
        current_dist = round(sqrt(current_dist), sim_param.round_value)
#        if __name__ == '__main__':
#            print('\ncurrent_dist', eef_pos, current_obj_pos, current_dist)

        dist_values = self.sections.values()
        if current_dist >= self.max_value:
            return self.section_name+str(len(dist_values)-1)
        elif current_dist <= self.min_value:
            return self.section_name+str(0)
        
        for pos in range(0,len(dist_values)):
            if  current_dist >= dist_values[pos][0] \
                and \
                current_dist < dist_values[pos][1]:
                    return self.section_name+str(pos)       
                                     
        return current_dist


'''
Compute distance
'''
def compute_distance(eef_pos, current_obj_pos,
                     current_dist): ## value of the discretization
                     
    round_init_vector = [round(x,sim_param.round_value) for x in eef_pos]
    round_final_vector = [round(x,sim_param.round_value) for x in current_obj_pos]                     
                     
    return current_dist.compute_section(round_init_vector, 
                                        round_final_vector)
 
'''
Test
'''
if __name__ == '__main__':
    discr_sections = Sections(0,
                              sim_param.circumference_radio,
                              sim_param.nb_min_distance_sections)
#    discr_sections = Sections(
#                          0, 
#                          2*(sim_param.circumference_radio**2),
#                          sim_param.nb_min_distance_sections)   
                              
    discr_sections.print_me()
    print(discr_sections.compute_section([0.65, 0.100384], [0.649999, 0.100001, 0]))

#    print(compute_distance([.0, 0.67], sim_param.obj_pos[:-1]))
    
    