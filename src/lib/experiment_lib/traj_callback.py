# -*- coding: utf-8 -*-
"""
@author: maestre
"""

from __future__ import print_function

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
lib_path = os.path.realpath(os.path.abspath(os.path.join('..', 'a2l_core_lib')))
sys.path.append(lib_path)
import ros_services
import simulation_parameters as sim_param

if sim_param.discr_hand_coded:
    import discretize_orientation_hand_coded as discr_orien
    import discretize_distance_hand_coded as discr_dist
else:
    import discretize_orientation_sections as discr_orien
    import discretize_inclination_sections as discr_inclin
    import discretize_distance_sections as discr_dist
    import discretize_effect as discr_effect

import discretize_move as discr_move
import environment_delta as delta

import rospy
from std_msgs.msg import Float64MultiArray


'''
callback class
'''
class Traj_callback():
    def __init__(self, 
                 simulated_traj,
                 sim_obj_moved,
                 current_orien, current_inclin, current_dist,
                 below_box):

        self.current_orien = current_orien
        self.current_inclin = current_inclin
        self.current_dist = current_dist
                     
        self.eef_traj_vector = [] # eef_traj_vector ## [[eef_x, eef_y, eef_z]]
        self.obj_traj_vector = [] #obj_traj_vector ## [[obj_x, obj_y, obj_z]]
        self.delta_vector = [] # delta_vector ## [[orientation, inclionation, distance, next_mov_discr]]
        self.traj_inferred_discr_delta_vector = [] # inf_dicr ## [[effect, idem]]
        self.delta_class_vector = [] #delta_class_vector
        self.obtained_effect = ""
        self.nb_executed_deltas = 0
        self.traj_res = False

        if sim_obj_moved:        
            ## subscribe to feedback topic
#            rospy.init_node('listener', anonymous=True)
            self.sub = rospy.Subscriber(sim_param.feedback_topic, 
                                       Float64MultiArray, 
                                       self.execute_traj_callback,
                                       queue_size=1)
#            self.sub_up = True        
                    
            ## avoid touching table in REAL ROBOT
            if not sim_param.real_robot or (sim_param.real_robot and not below_box):
                ## execute trajectory
                simulated_traj_vector = [i for el in simulated_traj for i in el] ## [float]
                res_exec = ros_services.call_trajectory_motion(sim_param.feedback_window, 
                                                               simulated_traj_vector)
#                rospy.spin()
                if sim_param.debug_infer:
                    if res_exec:
                        print("FINISHED EXECUTION IN ROS !! ")
                    else:
                        print("FAILED EXECUTION IN ROS !! ")
            else:
                print("-----------------> TRAJECTORY NOT EXECUTED !! TOUCHING THE TABLE!!! ")         
            
            self.sub.unregister()

    def execute_traj_callback(self, feedback_data):
#        print("\ntraj_callback ---> execute_traj_callback function", feedback_data.data)
#        print('traj_callback ---> feedback_data size', len(feedback_data.data))

        ''' Read feedback '''
        self.nb_executed_deltas = len(feedback_data.data)/6
        pos = 0
        batch_size = 6 * sim_param.feedback_window
        feedback_wp_vector = [] 
        feedback_obj_vector = []
        for pos in range(0, len(feedback_data.data), batch_size):
            eef_pos_x = feedback_data.data[pos+0]
            eef_pos_y = feedback_data.data[pos+1]
            eef_pos_z = feedback_data.data[pos+2]
            feedback_wp_vector.append([eef_pos_x,
                                       eef_pos_y,
                                       eef_pos_z])

            obj_pos_x = feedback_data.data[pos+3]
            obj_pos_y = feedback_data.data[pos+4]
            obj_pos_z = feedback_data.data[pos+5]                
            feedback_obj_vector.append([obj_pos_x,
                                        obj_pos_y,
                                        obj_pos_z])                

        self.eef_traj_vector = feedback_wp_vector
        self.obj_traj_vector = feedback_obj_vector
        
#        print("traj_callback ---> final eef pos", self.eef_traj_vector[-1])
#        print("traj_callback ---> final obj pos", self.obj_traj_vector[-1])
                
#        ''' Check if object moved'''
        real_obj_moved = False
        thrs = sim_param.obj_moved_threshold
        initial_obj_pos = self.obj_traj_vector[0]
        final_obj_pos = self.obj_traj_vector[-1]
  
        real_obj_moved = \
            (abs(initial_obj_pos[0] - final_obj_pos[0]) +
             abs(initial_obj_pos[1] - final_obj_pos[1]) +
             abs(initial_obj_pos[2] - final_obj_pos[2])) >= thrs                
#        print('real_obj_moved', real_obj_moved)
             
        if sim_param.debug_infer:
            print("obj_pos", initial_obj_pos, final_obj_pos)
        if real_obj_moved :
            ''' If object touched, compute result '''                                    
            self.obtained_effect = discr_effect.compute_effect(initial_obj_pos,
                                                               final_obj_pos)                    
            ''' Store delta info '''
            self.compute_traj_info(feedback_wp_vector, 
                                   feedback_obj_vector,
                                   real_obj_moved)

            for tmp_delta in self.delta_vector: 
                ## in discr dataset file : 
                self.traj_inferred_discr_delta_vector.append(
                    [self.obtained_effect,  ## effect
                     tmp_delta[0],  ## move 
                     tmp_delta[1],  ## distance 
                     tmp_delta[2],  ## orientation
                     tmp_delta[3]]) ## inclination
        else:
            if sim_param.debug_infer:    
                print('------------> BOX MOVE', 
                      abs(initial_obj_pos[0] - final_obj_pos[0]) +
                      abs(initial_obj_pos[1] - final_obj_pos[1]) +
                      abs(initial_obj_pos[2] - final_obj_pos[2]))                     
    '''
    a
    '''
    def compute_traj_info(self, 
                          feedback_wp_vector, 
                          feedback_obj_vector,
                          real_obj_moved):
        if sim_param.debug_infer:
            print("executing compute_traj_info function")  
        for pos in range(len(feedback_wp_vector)-1):
            ## store raw delta also as delta class              
            current_delta_class = delta.Delta(
                        self.obtained_effect,
                        feedback_wp_vector[pos][0],
                        feedback_wp_vector[pos][1],
                        feedback_wp_vector[pos][2],
                        feedback_wp_vector[pos+1][0],
                        feedback_wp_vector[pos+1][1],
                        feedback_wp_vector[pos+1][2],
                        [feedback_obj_vector[pos][0],
                        feedback_obj_vector[pos][1],
                        feedback_obj_vector[pos][2],
                        feedback_obj_vector[pos+1][0],
                        feedback_obj_vector[pos+1][1],
                        feedback_obj_vector[pos+1][2]])
            self.delta_class_vector.append(current_delta_class)
            
            ## compute and store discretized values
            move = discr_move.compute_move_discr(
                        [feedback_wp_vector[pos][0],
                        feedback_wp_vector[pos][1],
                        feedback_wp_vector[pos][2]],
                        [feedback_wp_vector[pos+1][0],
                        feedback_wp_vector[pos+1][1],
                        feedback_wp_vector[pos+1][2]])
                        
            distance = discr_dist.compute_distance(
                        [feedback_wp_vector[pos][0],
                        feedback_wp_vector[pos][1],
                        feedback_wp_vector[pos][2]],
                        [feedback_wp_vector[pos+1][0],
                        feedback_wp_vector[pos+1][1],
                        feedback_wp_vector[pos+1][2]],
                        self.current_dist)             
    
            orientation = discr_orien.compute_orientation_discr(
                        [feedback_wp_vector[pos][0],
                        feedback_wp_vector[pos][1]],
                        [feedback_wp_vector[pos+1][0],
                        feedback_wp_vector[pos+1][1]],
                        self.current_orien)
                             
            if sim_param.inclination_param:
                inclination = discr_inclin.compute_inclination_discr(
                            [feedback_wp_vector[pos][0],
                            feedback_wp_vector[pos][1],
                            feedback_wp_vector[pos][2]],
                            [feedback_wp_vector[pos+1][0],
                            feedback_wp_vector[pos+1][1],
                            feedback_wp_vector[pos+1][2]],
                            self.current_inclin)
                self.delta_vector.append([move, distance, orientation, inclination]) 
            else:
                self.delta_vector.append([move, distance, orientation]) 
            
        if sim_param.debug_infer:
            if sim_param.inclination_param:
                print("delta_discr", [move, distance, orientation, inclination])
            else:
                print("delta_discr", [move, distance, orientation])