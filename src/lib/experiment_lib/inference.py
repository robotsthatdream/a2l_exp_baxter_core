# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

import numpy as np
import random
from collections import OrderedDict

import os, sys
run_path = os.path.realpath(os.path.abspath(os.path.join('..', '..')))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Find pos of value given label 
Output: pos
'''
def findPosLabel(rand_var,label):
    label_found = False
    pos = 0
    while not label_found and pos < rand_var.domainSize():
        if rand_var.label(pos) == label:
            label_found = True
            return pos
        pos += 1

    if not label_found:
        return -1
        
'''
Infere next movement

bn = current bn
ie = inference setup
discr_values = 

'''
def infere_mov(bn, ie, node_names, node_values):    

    same_prob = False

    ''' Add soft evidence '''        
    ## generate dict with current node values to apply
    values_dict = OrderedDict()
    for i in range(len(node_names)): 
        current_node = node_names[i]
        current_value = node_values[i]
        values_dict[current_node] = current_value

    if sim_param.debug_infer:
        for i in values_dict:
            print (i, values_dict[i])

    ## transform each node values for the correspondant array 
    ## with 1 in the position of the current node value
    ## ej : if the possible node values are a b c, and we want to set c
    ## then it is 0 0 1
    for key in values_dict:        
        random_var = bn.variable(bn.idFromName(key))
        pos = findPosLabel(random_var,values_dict[key])
        if pos==-1:
            if sim_param.debug_infrt:
                print("Label not found for",str(key),values_dict[key])
        else:
            tmp=np.zeros(random_var.domainSize())
            for p in range(len(tmp)):
                if p==pos:
                    tmp[p] = 1
                else:
                    tmp[p] = 0
            values_dict[key] = tmp.tolist()
        
    ie.setEvidence(values_dict)
    
    ''' Infere next movement to be done '''    
    ie.makeInference()
    
    ## get posterior    
    posterior = ie.posterior(bn.idFromName('move'))
#    if sim_param.debug:
#    print('\n')
#    print(posterior)
#    print(posterior[0])
#    print(posterior[-1])
#    print(type(posterior))
#    print(type(posterior[0]))
#    print(posterior.variablesSequence()[0])
#    print(type(posterior.variablesSequence()[0]))
#    print(posterior.variablesSequence()[0].labels())
    

#    ## check if all values of 'move' have been generated
    move_var = bn.variable(bn.idFromName('move'))    
    move_nb_var = move_var.domainSize()
#    if move_nb_var != len(sim_param.move_values):
#        print('ERROR - infere_mov- Some move value is not available in the dataset')
#        sys.exit()

    ## get used moves
    move_list = []
    for pos in range(move_nb_var):
        move_list.append((posterior.variablesSequence())[0].label(pos))
        pos += 1

    ## select value of next move
    ## check if some prob is very high, or if 2 variables have a similar value
    posterior_list = []
    for i in range(len(move_list)):
        posterior_list.append(posterior[i])
        
    if all(x==posterior_list[0] for x in posterior_list):## no prob inferred
        posterior_pos = random.randint(0,len(posterior_list)-1)
        posterior_value = posterior_list[posterior_pos]
        same_prob = True
    else:
        max_value = max(posterior_list)
        posterior_pos = posterior_list.index(max_value)
        posterior_value = posterior_list[posterior_pos]
        max_pos = posterior_list.index(max_value)
        
#        ## choose posterior value
#        if max_value > 0.9: ## very high
#            posterior_pos = max_pos              
#        else: ## several probs
#            posterior_list[max_pos] = 0
#            second_max_value = max(posterior_list)
#            second_max_pos = posterior_list.index(max(posterior_list))
#
##            if max_value > 2*second_max_value: ## far probs
##                posterior_pos = max_pos
##            else: ## close probs
##                posterior_pos = random.choice([max_pos, second_max_pos])
#
#            if max_value == second_max_value:
#                posterior_pos = random.choice([max_pos, second_max_pos])
#            else:
#                posterior_pos = max_pos

        ## assign posterior value            
        if posterior_pos == max_pos:
            posterior_value = max_value
#        else:
#            posterior_value = second_max_value
            
    move_value = (posterior.variablesSequence())[0].label(posterior_pos)    
    if sim_param.debug:
        print("Best posterior of move is", move_value.upper(),\
            "with prob", posterior_value, "in position", posterior_pos, '\n')      
    
    return move_value, round(posterior_value, 5), int(move_nb_var), same_prob