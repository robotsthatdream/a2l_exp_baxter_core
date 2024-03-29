# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

class Wp():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_z(self):
        return self.z
        
class Delta():
    def __init__(self, 
                 effect,
                 wp_init_x, wp_init_y, wp_init_z,
                 wp_final_x, wp_final_y, wp_final_z,
                 next_obj_pos_vector): ## of cube
        self.effect = effect
        self.wp_init = Wp(wp_init_x, wp_init_y, wp_init_z)
        self.wp_final = Wp(wp_final_x, wp_final_y, wp_final_z)
        self.current_next_obj_pos_vector = next_obj_pos_vector

    def get_effect(self):
        return self.effect
        
    def get_wp_init(self):
        return self.wp_init
    def get_wp_final(self):
        return self.wp_final

    ## current_next_obj_pos_vector  = [OPinit OPfinal]
    ## OPinit = OPfinal = x y z
    def get_obj_init(self):
        obj_info = self.current_next_obj_pos_vector
        return [obj_info[0], obj_info[1], obj_info[2]]
    def get_obj_final(self):
        obj_info = self.current_next_obj_pos_vector
        return [obj_info[3], obj_info[4], obj_info[5]]
        
    def set_effect(self, effect):
        self.effect = effect

    def update_obj_initial(self, obj_info):
        self.current_next_obj_pos_vector[0] = obj_info[0]
        self.current_next_obj_pos_vector[1] = obj_info[1]
        self.current_next_obj_pos_vector[2] = obj_info[2]

    def update_obj_final(self, obj_info):
        self.current_next_obj_pos_vector[3] = obj_info[0]
        self.current_next_obj_pos_vector[4] = obj_info[1]
        self.current_next_obj_pos_vector[5] = obj_info[2]
        
    def print_me(self):
        print('\neffect', self.effect)
        print('wp_init', self.wp_init.get_x(), self.wp_init.get_y(), 
              self.wp_init.get_z())
        print('wp_final', self.wp_final.get_x(), self.wp_final.get_y(), 
              self.wp_final.get_z())
#        print('obj_init', self.obj_init.get_x(), self.obj_init.get_y(), 
#              self.obj_init.get_z())
#        print('obj_final', self.obj_final.get_x(), self.obj_final.get_y(), 
#              self.obj_final.get_z())
#        print('obj_moved', self.obj_moved, '\n')
        for obj_id in range(len(sim_param.obj_name_vector)):
            obj_info = self.current_next_obj_pos_vector#[obj_id]
            print('Object :',obj_id)
            print('Current pos:', obj_info[0], obj_info[1], obj_info[2])
            print('Next pos:', obj_info[3], obj_info[4], obj_info[5])
            
            
            
            