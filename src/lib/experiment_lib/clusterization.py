from __future__ import print_function

import numpy as np
import random
import time
#from contextlib import contextmanager
from pyxmeans import _minibatch
#from pyxmeans.mini_batch import MiniBatch
from pyxmeans.xmeans import XMeans
import pylab as py
from sklearn.cluster import MiniBatchKMeans, KMeans

import dataset_generation as dataset

def TimerBlock(name):
    start = time.time()
    yield
    end = time.time()
    print ("%s took %fs" % (name, end-start))


'''
Main
'''
if __name__ == '__main__':
    
    ## read trajs
    filename = '/home/maestre/indigo/baxter_ws/src/a2l_exp_baxter_actions/src/generated_datasets/directed_dataset.csv'
    raw_dataset = dataset.read_dataset(filename)
    
    ## extract obj final positions
    initial_obj_pos = [raw_dataset[0].get_obj_init().get_x(), \
                        raw_dataset[0].get_obj_init().get_y(), \
                        raw_dataset[0].get_obj_init().get_z()]
#    print(initial_obj_pos)    

    final_obj_pos_vector = []
    for delta in raw_dataset:
        current_obj_pos = [delta.get_obj_final().get_x(),
                                     delta.get_obj_final().get_y()] #,
#                                     delta.get_obj_final().get_z()]
        
        if (abs(initial_obj_pos[0] - current_obj_pos[0]) > 0.01) or \
            (abs(initial_obj_pos[1] - current_obj_pos[1]) > 0.01) :
#            print(current_obj_pos)                                     
            final_obj_pos_vector.append(current_obj_pos)                
    
    ## clusterize them
    x_means = True
    
    ''' X means '''    
    py.figure()
    if x_means:
                
        k = 5
        max_iter = 2
        n_samples = 2
        k_init = 2 #int(k*0.65) 
        
        ## small fake dataset
        if 0:        
            final_obj_pos_vector = [[random.randint(0,1) + random.uniform(-0.3,0.3),
                                     random.randint(0,1) + random.uniform(-0.3,0.3)] 
                                     for i in range(100)]

        ## big fake dataset                 
        if 0:
            final_obj_pos_vector = [[random.randint(0,3) + random.uniform(-0.2,0.2),
                                     random.randint(0,3) + random.uniform(-0.2,0.2)] 
                                     for i in range(1000)]
            k = 16
            max_iter = 100
            n_samples = k*2
            k_init = 14 #int(k*0.65)

        data = np.array(final_obj_pos_vector) 
        
        mxmt = XMeans(k_init, verbose=True)
        mxmt.fit(data)
        clusters_xmeans = mxmt.cluster_centers_
        print("Num Clusters: ", len(clusters_xmeans))
        print("BIC: ", _minibatch.bic(data, clusters_xmeans))
        print("Variance: ", _minibatch.model_variance(data, clusters_xmeans))
    #    print("RMS Error: ", error(actual_data, clusters_xmeans))
        print("")
        py.title("x - means")          
        py.scatter(data[:,0], data[:,1], alpha=0.25, label="data")
        py.scatter(clusters_xmeans[:,0], clusters_xmeans[:,1], c='red', 
                   s=75, alpha=0.4, label="xmeans")

    else:
        data = np.array(final_obj_pos_vector)
        random_state = 5
        y_pred = MiniBatchKMeans(n_clusters=4, random_state=random_state).fit_predict(data)
        py.title("k - means")  
        py.scatter(data[:, 0], data[:, 1], c=y_pred, label='data')        
        py.plot(0.65, 0.1, 's', markersize = 15, color = 'black') 
        
    py.legend()
    py.tight_layout()
    py.show()    
