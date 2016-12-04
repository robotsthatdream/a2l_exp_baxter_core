#!/usr/bin/env python

"""
Created on Mon Nov 30 15:00:16 2015

@author: maestre
"""
import os, sys
lib_path = os.path.abspath(os.path.join('.', 'lib'))
sys.path.append(lib_path)

import pyAgrum as agrum

import trajectory_gen as tg
import trajectory_gen_random as tg_random
import inference as inf
import validation as check

'''
Main
'''
#if __name__ == "__main__":
#    if len(sys.argv) != 3:
#        print("ERROR - Wrong input values")
#        print("USE : python3 createBN.py nb_dossier nb_file eef_x eef_y eef_z finalobj_pos_x finalobj_pos_y finalobj_pos_z")
#        exit        
#    nb_dossier = "dataset/" + str(sys.argv[1])
#    nb_archive = str(sys.argv[2])

'''
Create directed or random dataset
'''
def create_dataset(directed_dataset):

    ## random obj pos
    obj_pos = [0,0,0]
    obj_side = 0.3    
    
    ## naive exploration -> initial dataset    
    ## generate dataset of trajectories
#    directed_dataset = True
    if directed_dataset:
        tg.create_discr_trajs(obj_pos, obj_side)
        print('Initial DIRECTED dataset generated')
    else:
        tg_random.create_discr_trajs(obj_pos, obj_side)
        print('Initial RANDOM dataset generated')
    
'''
Learn BN with a specific structure learning algorithm
'''
def learn_bn(learn_algo):
    ## create and learn BN, and setup inference
    dataset_path = 'discr_wps.csv'
    bn = inf.learn_bn(learn_algo, dataset_path)    
    bn_url = "BN.bif"
    agrum.saveBN(bn,bn_url)
    print("BN saved in " + bn_url)
    
    ## BN validation : identify unknown effects
    ## effect = [goal_value, vector_orient_value]
#    unk_aff_vector, nb_aff_dataset, nb_aff_total = check.find_unk_affordances(bn_url)
#
#    if nb_aff_total != 0:
#        print('\nRESULT:')
##        print(nb_aff_total, 'possible affordances in the experiment')
##        print(nb_aff_dataset, 'possible affordances in the dataset')
#        print(len(unk_aff_vector),'affordances not learned yet')
#    else :
#        print('\nNOT VALID DATASET. Only',nb_aff_dataset,'lines read')
        
    return bn
    