#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@author: maestre
"""
import rospy
from a2l_exp_baxter_actions.srv import *
import time

import os
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

#from gazebo_msgs.srv import GetModelState

'''
a
'''
def update_nb_init_pos():
    rospy.set_param("nb_init_pos", sim_param.nb_min_init_pos)

'''
a
'''
def call_generate_directed_dataset(dataset_type):
    service_name = 'a2l/generate_directed_dataset'
    rospy.wait_for_service(service_name)
    try:
        if sim_param.debug_services:
            print('--> CALL SERVICE generate_directed_dataset')         
        if dataset_type == 'directed':
            type_name = Generatedirecteddataset          
        else:
            print('TBD creating not directed dataset')
            sys.exit()
            
        create_dataset = rospy.ServiceProxy(service_name,
                                            type_name)
        resp = create_dataset()
        return resp.path                
    except rospy.ServiceException, e:
        print ("Service call to generate_directed_dataset failed: %s"%e)

'''
a
'''
def call_get_eef_pose(eef_name):
    service_name = 'a2l/get_eef_pose'
    rospy.wait_for_service(service_name)
    try:
        if sim_param.debug_services:
            print('--> CALL SERVICE get_eef_pose')        
        type_name = Geteefpose
        get_eef_pose = rospy.ServiceProxy(service_name, 
                                          type_name)
        resp = get_eef_pose(eef_name)
        return resp.pose
    except rospy.ServiceException, e:
        print ("Service call to get_eef_pose failed: %s"%e)
        
'''
a
'''
def call_get_model_state(model_name):
    service_name = 'a2l/get_model_state'
    rospy.wait_for_service(service_name)
    try:        
        if sim_param.debug_services:
           print('--> CALL SERVICE get_model_state')        
        type_name = Getmodelstate
        get_model_state = rospy.ServiceProxy(service_name, 
                                             type_name)
        resp = get_model_state(model_name)
        print(resp)
        return resp.model_state        
    except rospy.ServiceException, e:
        print ("Service call to get_model_state failed: %s"%e) 
        

#def call_get_model_state(model_name):
#    service_name = 'gazebo/get_model_state'
#    rospy.wait_for_service(service_name)
#    try:        
#        if sim_param.debug_services:
#           print('--> CALL SERVICE get_model_state')        
#        gms = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
#        resp = gms(model_name,'base')
#        print(resp)
#        return [resp.pose.position.x, 
#                resp.pose.position.y,
#                resp.pose.position.z]
#    except rospy.ServiceException, e:
#        print ("Service call to get_model_state failed: %s"%e) 
        

'''
a
'''
def call_execute_delta_motion(delta_x, delta_y, delta_z):
    service_name = 'a2l/delta_motion'
    rospy.wait_for_service(service_name)
    try:        
        if sim_param.debug_services:
            print('--> CALL SERVICE execute_delta_motion')        
        type_name = Deltamotion
        execute_delta_motion = rospy.ServiceProxy(service_name, 
                                             type_name)
        resp = execute_delta_motion(delta_x, delta_y, delta_z)
        time.sleep(2)
        return resp.success        
    except rospy.ServiceException, e:
        print ("Service call to execute_delta_motion failed: %s"%e)     
        

'''
a
'''
def call_trajectory_motion(feedback_frequency, trajectory):
    service_name = 'a2l/trajectory_motion'
    rospy.wait_for_service(service_name)
    try:
        if sim_param.debug_services:
            print('--> CALL SERVICE execute_trajectory')        
        type_name = Deltatrajectory
        execute_traj = rospy.ServiceProxy(service_name, 
                                                  type_name)
        resp = execute_traj(float(feedback_frequency),
                            trajectory)
        return resp.success        
    except rospy.ServiceException, e:
        print ("Service call to execute_delta_motion failed: %s"%e) 
        
'''
a
'''
def call_move_to_initial_position(nb_initial_pos):
    service_name = 'a2l/move_to_initial_position'
    rospy.wait_for_service(service_name)
    try:
        if sim_param.debug_services:
            print('--> CALL SERVICE move_to_initial_position') 
        type_name = Movetoinitpos
        move_to_initial_position = rospy.ServiceProxy(service_name, 
                                             type_name)
        resp = move_to_initial_position(nb_initial_pos)
        time.sleep(1)
        return resp.success        
    except rospy.ServiceException, e:
        print ("Service call to move_to_initial_position failed: %s"%e)  
        
'''
a
'''
def call_restart_world(element):
    service_name = 'a2l/restart_world'
    rospy.wait_for_service(service_name)
    try:        
        if sim_param.debug_services:
            print('--> CALL SERVICE restart_world') 
        type_name = Restartworld
        restart_world = rospy.ServiceProxy(service_name, 
                                             type_name)
        resp = restart_world(element)      
        return resp.success        
    except rospy.ServiceException, e:
        print ("Service call to restart_world failed: %s"%e)    
        
if __name__  == "__main__":
    print("cube :", call_get_model_state("cube"))
    
#    print("create directed dataset", 
#          call_generate_directed_dataset('directed'))
#    
#    print("call_get_eef_pose right", call_get_eef_pose('right'))
    
#    print("call_get_eef_pose left", call_get_eef_pose('left'))
    
#    print("call_execute_delta_motion", 
#          call_execute_delta_motion(0.1, 
#                                    -0.05, 
#                                    0))
    
#    print("call_move_to_initial_position left", 
#          call_move_to_initial_position(1))
    
#    print("call_trajectory_motion", 
#           call_trajectory_motion(2, [0.65,0.2,0.1,0.5,0,0,0.7,0.3,0.2]))    
    
   




     
    