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

import os, sys
run_path = os.path.abspath(os.path.join('..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

class Sections:

    ''' Initialize min value '''
    def __init__(self, min_angle, max_angle, nb_sections):
        self.section_name = 'section_'
        self.min_angle = min_angle
        self.max_angle = max_angle        
        tmp_sections = np.linspace(min_angle, max_angle, nb_sections + 1)
        ## Label degree sections
        self.sections_degrees = collections.OrderedDict()
        pos = 0
        for value in tmp_sections[:-1]:
            self.sections_degrees[self.section_name+str(pos+1)] = \
                [math.degrees(value), math.degrees(tmp_sections[pos+1])]
            pos += 1
                
        ## Label radian sections
        self.sections_radians = collections.OrderedDict()
        pos = 0
        for value in tmp_sections[:-1]:
            self.sections_radians[self.section_name+str(pos+1)] = \
                [value, tmp_sections[pos+1]]
            pos += 1
        
    ''' Get the sections '''
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

        if (current_angle < self.min_angle or 
            current_angle > self.max_angle):            
            print("ERROR - get_section_angle : The value", current_angle, \
                    "is out of the range [", self.min_angle, ", ", \
                    self.max_angle, "]")
            sys.exit() 

        radian_values = self.sections_radians.values()

        if current_angle == self.max_angle:
            return self.section_name+str(len(radian_values))
        elif current_angle == self.min_angle:
            return self.section_name+str(0)
        for pos in range(0,len(radian_values)-1):
            if current_angle > radian_values[pos] and \
                    current_angle <= radian_values[pos+1]:
                return self.section_name+str(pos)

    ''' Compute the section of a given pos '''
    def compute_section(self,pos_init, pos_final):
        angle = angle_2_abscise(pos_init, pos_final)
        return self.get_section_angle(angle)

def angle_2_abscise(pos_init, pos_final):
    angle = math.atan2(pos_init[1] - pos_final[1], 
                       pos_init[0] - pos_final[0])
#    ## to transpose from [-Pi, Pi] to [0, 2*Pi]
#    if(pos_init[0]-pos_final[0]<0):
#       angle = angle + math.pi
    return angle
                    
if __name__ == "__main__":
    min_angle = -math.pi
    max_angle = math.pi
    nb_sections = 8    
    discr_sections = Sections(min_angle, max_angle, nb_sections)
    
    angle_format = 'radians'
    discr_sections.print_me(angle_format)
    
    print()    
    
    angle_format = 'degrees'
    discr_sections.print_me(angle_format)
    
#    print(discr_sections.compute_section([0,0], [1,1]))
    