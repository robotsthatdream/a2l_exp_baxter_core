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
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

class Sections:

    def __init__(self, inclin_min_angle, inclin_max_angle, nb_sections):
        self.section_name = 'inclin_'
        self.inclin_min_angle = inclin_min_angle
        self.inclin_max_angle = inclin_max_angle        
        
        ## create sections
        self.raw_sections = np.linspace(inclin_min_angle, inclin_max_angle, nb_sections + 1)
        ## if some need to be removed
        if sim_param.discr_random:
            nb_random_values = len(self.raw_sections)//nb_sections
            for v in range(nb_random_values):
                to_remove = random.randint(1,len(self.raw_sections)-2)
                self.raw_sections = np.delete(self.raw_sections, to_remove)        
        self.raw_sections = [round (v, sim_param.round_value) 
                        for v in self.raw_sections]
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
        
        for tmp_d_key, tmp_d_value in tmp_list.items():
            print(tmp_d_key, "-->", tmp_d_value)

    ''' Get the name of the discretized section for an angle '''
    def get_section_angle(self,current_angle):            
            
        if (current_angle < self.inclin_min_angle or 
            current_angle > self.inclin_max_angle):
            print("ERROR - get_section_angle : The value", current_angle, \
                    "is out of the range [", self.inclin_min_angle, ", ", \
                    self.inclin_max_angle, "]")
            sys.exit() 

        radian_values = self.sections_radians.values()

        if current_angle == self.inclin_max_angle:
            return self.section_name+str(len(radian_values)-1)
        elif current_angle == self.inclin_min_angle:
            return self.section_name+str(0)
        
        current_angle = round(current_angle, sim_param.round_value)
        for pos in range(0,len(radian_values)):
            if  current_angle >= radian_values[pos][0] \
                and \
                current_angle < radian_values[pos][1]:
                    return self.section_name+str(pos)

    ''' Compute the section of a given pos '''
    def compute_section(self,pos_init, pos_final):
        angle = angle_2_ordinate(pos_init, pos_final)
        return self.get_section_angle(angle)

'''
Compute angle of a vector respect to the vertical axis in range [0, Pi/2]
http://mathworld.wolfram.com/SphericalCoordinates.html
'''
def angle_2_ordinate(pos_init, pos_final):
    vector = [pos_final[0] - pos_init[0],
              pos_final[1] - pos_init[1], 
              pos_final[2] - pos_init[2]]
    xy = math.sqrt(vector[0]**2 + 
                  vector[1]**2)
    z = vector[2]
    angle = np.arctan2(xy, z)
#    print(pos_init[0:3], pos_final[0:3], round(angle* 180 / np.pi,2))
    return angle

'''
Given two positions return the inclination of the discretized vector 
between them 
'''
def compute_inclination_discr(pos_init_vector, pos_final_vector,
                              current_orien):

    round_init_vector = [round(x,sim_param.round_value) for x in pos_init_vector]
    round_final_vector = [round(x,sim_param.round_value) for x in pos_final_vector]

    return current_orien.compute_section(round_init_vector, 
                                         round_final_vector)

'''
Test
'''
    
if __name__ == "__main__":
    sections = Sections(sim_param.inclin_min_angle, 
                        sim_param.inclin_max_angle, 
                        sim_param.nb_min_inclination_sections)    
    
    angle_format = 'degrees'
    sections.print_me(angle_format)
    
#    print(sections.get_section_angle(sim_param.inclin_max_angle))
    
#    print(angle_2_ordinate([0,0,0], [0, 0, 1])* 180 / np.pi)
    
#    print()    
#    
#    angle_format = 'degrees'
#    sections.print_me(angle_format)   
#
#    print()
    
#    print(compute_inclination_discr([-1.5,1.5,1.5], [0, 0, 0], sections))    
    
#                                   init       final
#    print(compute_inclination_discr([0, 0, 0], [0.99999,0.99999,0], sections))
    print(compute_inclination_discr([0.65, 0.1, -0.14], [0.87, 0.1, 0.12], sections))
    
#    print(compute_inclination_discr([1,.1,.1], [0, 0, 0], sections))    
#    print(compute_inclination_discr([-1,.1,.1], [0, 0, 0], sections))    

#    print(compute_inclination_discr([0.6460916809755015, 
#                                     -0.10103936345923037, 
#                                     0.9544409157082219], 
#                                    [0.6499998645735556, 
#                                     0.09999986267875577, 
#                                     0.8250000037335137], sections))


#    print()
#    
#    print(sections.compute_section([0, 0], [1,1]))
#    print(sections.compute_section([0, 0], [1,-1]))
#
#    print(sections.compute_section([0, 0], [-1,1]))
#    print(sections.compute_section([0, 0], [-1,-1]))
    

    