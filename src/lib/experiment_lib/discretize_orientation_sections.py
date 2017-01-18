# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 10:06:47 2016

@author: maestre
"""
from __future__ import print_function

import math
#import matplotlib.pyplot as plt
import numpy as np
import collections
import random

import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

class Sections:

    def __init__(self, min_angle, max_angle, nb_sections):        
        self.section_name = 'orien_'
        self.orien_min_angle = min_angle
        self.orien_max_angle = max_angle
        
        ## create sections
        self.raw_sections = np.linspace(min_angle, max_angle, nb_sections + 1)
        self.raw_sections = [round (v, sim_param.round_value) 
                             for v in self.raw_sections]
        
#        print(self.orien_min_angle, self.orien_max_angle, self.raw_sections)
        
        ## if some need to be removed
        if sim_param.discr_random:
            nb_random_values = len(self.raw_sections)//nb_sections
            for v in range(nb_random_values):
                to_remove = random.randint(1,len(self.raw_sections)-2)
                self.raw_sections = np.delete(self.raw_sections, to_remove)        
        
        ## Label degree sections
        self.sections_degrees = collections.OrderedDict()
        pos = 0
        for value in self.raw_sections[:-1]:
            self.sections_degrees[self.section_name+str(pos)] = \
                [math.degrees(value), 
                 math.degrees(self.raw_sections[pos+1])]
            pos += 1
                
        ## Label radian sections
        self.sections_radians = collections.OrderedDict()
        pos = 0
        for value in self.raw_sections[:-1]:
            self.sections_radians[self.section_name+str(pos)] = \
                [value, self.raw_sections[pos+1]]
            pos += 1

    ''' Get the raw sections (list) '''
    def get_raw_sections(self):
        return self.raw_sections   
        
    ''' Get the sections (dictionary) '''
    def get_sections(self, angle_format):
        if angle_format == 'radians':
            return self.sections_radians
        elif angle_format == 'degrees':
            return self.sections_degreess
        else:
            print('ERROR - print_me - angle format unknown')
            sys.exit()

    ''' Print the discretized sections'''        
    def print_me(self, angle_format):
        if angle_format == 'radians':
            tmp_list = self.sections_radians
        elif angle_format == 'degrees':
            tmp_list = self.sections_degrees
        else:
            print('ERROR - print_me - angle format unknown')
            sys.exit()
        
        pos = 0
        for tmp_d_key, tmp_d_value in tmp_list.items():
            if pos == len(self.sections_degrees)-1:
                tmp_d_value[1] = math.degrees(self.orien_max_angle - sim_param.orien_offset)
#            print(tmp_d_key, "-->", tmp_d_value)
            if angle_format == 'radians':
                if pos == len(self.sections_degrees)-1:
                    print(tmp_d_key, "-->", [self.orien_min_angle - sim_param.orien_offset,
                                             self.orien_min_angle])
            else:
                if pos == len(self.sections_degrees)-1:
                    print(tmp_d_key, "-->", [math.degrees(self.orien_min_angle - sim_param.orien_offset),
                                             math.degrees(self.orien_min_angle)])
                
            pos += 1

    ''' Get the name of the discretized section for an angle '''
    def get_section_angle(self,current_angle):            

        if (current_angle < self.orien_min_angle - sim_param.orien_offset or 
            current_angle > self.orien_max_angle - sim_param.orien_offset):            
            print("ERROR - get_section_angle : The value", current_angle, \
                    "is out of the range [", 
                    self.orien_min_angle - sim_param.orien_offset, ", ", \
                    self.orien_max_angle - sim_param.orien_offset, "]")
            sys.exit()

        radian_values = self.sections_radians.values()

        if current_angle == self.orien_max_angle - sim_param.orien_offset:
            return self.section_name+str(len(radian_values)-1)
        elif current_angle == self.orien_min_angle - sim_param.orien_offset:
            return self.section_name+str(0)        
        elif current_angle >= self.orien_min_angle - sim_param.orien_offset and \
            current_angle < self.orien_min_angle:
            return self.section_name+str(len(radian_values)-1)    
        
#        current_angle = round(current_angle, sim_param.round_value)
        for pos in range(0,len(radian_values)):
            if  current_angle >= radian_values[pos][0] \
                and \
                current_angle < radian_values[pos][1]:
                    return self.section_name+str(pos)

    ''' Compute the section of a given pos '''
    def compute_section(self,pos_init, pos_final):
        angle = round(angle_2_abscise(pos_init, pos_final), sim_param.round_value)
        return self.get_section_angle(angle)

'''
Compute angle of a vector respect to the horizontal axis in range [-Pi, Pi]
'''
def angle_2_abscise(pos_init, pos_final):
    angle = math.atan2(pos_init[1] - pos_final[1], 
                       pos_init[0] - pos_final[0])
#    print(math.degrees(angle))
    return angle # + math.pi/2

'''
Given two positions return the orientation of the discretized vector 
between them 
'''
def compute_orientation_discr(pos_init_vector, pos_final_vector,
                              current_orien):

    round_init_vector = [round(x,sim_param.round_value) for x in pos_init_vector]
    round_final_vector = [round(x,sim_param.round_value) for x in pos_final_vector]

    return current_orien.compute_section(round_init_vector, 
                                         round_final_vector)

'''
Test
'''
    
if __name__ == "__main__":
    sections = Sections(sim_param.orien_min_angle, 
                        sim_param.orien_max_angle, 
                        sim_param.nb_min_orientation_sections)    
    
    angle_format = 'degrees'
    sections.print_me(angle_format)
    
#    print(sections.get_section_angle(sim_param.max_angle))
    
#    print()    
#    
#    angle_format = 'degrees'
#    sections.print_me(angle_format)   
#
#    print()
#    
#    print(compute_orientation_discr([0.649494, -0.0766049], 
#                                    [0.649216, -0.0441747], sections))
          
    print(compute_orientation_discr([0,0], ## up
                                    [0,1], sections))    
    print(compute_orientation_discr([0,0], ## left
                                    [-1,0], sections))                                        
    print(compute_orientation_discr([0,0], ## down
                                    [0,-1], sections))    
    print(compute_orientation_discr([0,0], ## right
                                    [1,0], sections))                                                                            
    
#    0.649494 -0.0766049 -0.0487123

#    print()
#    
#    print(sections.compute_section([0, 0], [1,1]))
#    print(sections.compute_section([0, 0], [1,-1]))
#
#    print(sections.compute_section([0, 0], [-1,1]))
#    print(sections.compute_section([0, 0], [-1,-1]))
    

    