"""
@author: maestre
"""

from __future__ import print_function
import pyAgrum as agrum

import os, sys
run_path = os.path.abspath(os.path.join('..'))
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
    if learn_algo == 'hand-coded':    
        learner.addMandatoryArc(0,2)    
        learner.addMandatoryArc(1,2)
        if sim_param.distance_param:
            learner.addMandatoryArc(3,2)
    elif learn_algo == 'hillclimbing':
        learner.useGreedyHillClimbing()
    elif learn_algo == 'tabu':
        learner.useLocalSearchWithTabuList()
    elif learn_algo == 'k2':
        learner.useK2([3,2,1,0])
        
    else:
        print('ERROR - learn_bn : there was a problem while selecting the learner')
        sys.exit()

    ''' Select score (BDEU by default)'''
    if sim_param.score_likelihood:
        learner.useScoreLog2Likelihood()
        learner.setMaxIndegree(2)
    
    if sim_param.score_bic:
        learner.useScoreBIC
        
    if sim_param.score_aic:
        learner.useScoreAIC
        
    bn=learner.learnBN()
    print("BN learned.\n", bn)
    return bn   
    
'''
Save BN
'''
def save_bn(bn, bn_url):
    agrum.saveBN(bn,bn_url)
    print("BN saved in " + bn_url)
    
if __name__ == "__main__":
    dataset_filepath = '../generated_files/directed_discr_wps.csv'
    #dataset_filepath = '../generated_files/random_discr_wps.csv'
    
    #learn_algo = 'hand-coded'    
    #learn_algo = 'hillclimbing'
    #learn_algo = 'k2'
    learn_algo_vector = ['hand-coded', 'hillclimbing', 'k2']
    
    for algo in learn_algo_vector:
        print("\n" + algo.upper())
        bn = learn_bn(dataset_filepath, algo)    
        bn_filepath = '../generated_files/BN_' + algo + '.bif'
        save_bn(bn, bn_filepath)
        
    print('Done.')