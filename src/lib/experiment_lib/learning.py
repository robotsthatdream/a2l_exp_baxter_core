# -*- coding: utf-8 -*-
"""
@author: maestre
"""
from __future__ import print_function

import pyAgrum as agrum

import os, sys
run_path = os.path.abspath(os.path.join('..', '..'))
sys.path.append(run_path)
import simulation_parameters as sim_param

'''
Create and learn BN, and setup inference
'''
def learn_bn(filepath, learn_algo):
    try:
        learner=agrum.BNLearner(filepath)
        
    except IOError as e:
        print ("learn_and_create_bn - I/O error({0}): {1}".format(e.errno, e.strerror))
        exit        
    
    ''' Select learning method '''
    if learn_algo == 'hard-coded':    
        learner.addMandatoryArc(0,3)    
        learner.addMandatoryArc(1,3)
        learner.addMandatoryArc(2,3)
        learner.addMandatoryArc(4,3)
        
    elif learn_algo == 'hillclimbing':
        learner.useGreedyHillClimbing()
    elif learn_algo == 'tabu':
        learner.useLocalSearchWithTabuList()
    elif learn_algo == 'k2':
        if sim_param.distance_param:
            learner.useK2([3,2,1,0])
        else:
            learner.useK2([2,1,0])
        
    else:
        print('ERROR - learn_bn : there was a problem while selecting the learner')
        sys.exit()

    ''' Select score (BDEU by default)'''
    if sim_param.score_likelihood:
        learner.useScoreLog2Likelihood()
        learner.useAprioriSmoothing()
    
    if sim_param.score_bic:
        learner.useScoreBIC()
        learner.useAprioriSmoothing()
        
        
    if sim_param.score_aic:
        learner.useScoreAIC()
        learner.useAprioriSmoothing()
        
    if sim_param.score_k2:
        learner.useScoreK2()
#        learner.useAprioriSmoothing()        
        
    bn=learner.learnBN()
    if sim_param.debug:
        print("BN learned.\n", bn)
    return bn   
    
'''
Save BN
'''
def save_bn(bn, bn_url):
    agrum.saveBN(bn,bn_url)
    if sim_param.debug:
        print("BN saved in " + bn_url)    