# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 11:20:05 2017

@author: maestre
"""

import numpy as np
import matplotlib.pyplot as plt
#from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import multivariate_normal
import math

#def multivariate_gaussian(pos, mu, Sigma):
#    """Return the multivariate Gaussian distribution on array pos.
#
#    pos is an array constructed by packing the meshed arrays of variables
#    x_1, x_2, x_3, ..., x_k into its _last_ dimension.
#
#    """
#    n = mu.shape[0]
#    Sigma_det = np.linalg.det(Sigma)
#    Sigma_inv = np.linalg.inv(Sigma)
#    N = np.sqrt((2*np.pi)**n * Sigma_det)
#    # This einsum call calculates (x-mu)T.Sigma-1.(x-mu) in a vectorized
#    # way across all the input variables.
#    fac = np.einsum('...k,...kl,...l->...', pos-mu, Sigma_inv, pos-mu)
#
#    return np.exp(-fac / 2) / N
#
## The distribution on the variables X, Y packed into pos.
#Z = multivariate_gaussian(pos, mu, Sigma)

'''
a
'''
def gaussian_discretization(obj_pos):

    # Our 2-dimensional distribution will be over variables X and Y
    N = 60
    X = np.linspace(-radius, radius, N)
    Y = np.linspace(-radius, radius, N)
    X, Y = np.meshgrid(X, Y)

    section_vector = []
    for nb_curr_orien in range(nb_min_orientation_sections):        
        
        ## Mean vector 
        mu = np.array([0., 0.])
        
        ## Covariance matrix
        sigma = np.array([[ 1 ,0], [0,  4]])
        
        # Pack X and Y into a single 3-dimensional array
        pos = np.empty(X.shape + (2,))
        pos[:, :, 0] = X
        pos[:, :, 1] = Y
        
        rv = multivariate_normal(mean=mu, cov=sigma)
        Z = rv.pdf(pos)
        
        if print_gaussian:
            
            # Create a surface plot and projected filled contour plot under it.
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            #ax.plot_surface(X, Y, Z, rstride=3, cstride=3, linewidth=1, antialiased=True,
            #                cmap='Blues')
            
            ax.contourf(X, Y, Z, zdir='z', offset=0, cmap='Blues')
            
            # Adjust the limits, ticks and view angle
            
            ax.set_zlim(-0.15,0.2)
            ax.set_zticks(np.linspace(0,0.2,5))
            
            #ax.view_init(27, -21)
            ax.view_init(90, 180) ## robot point of view
            
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            
            plt.show()
    
    return section_vector

'''
Compute distribution probality for a position [x,y]
'''
def check_pos_gaussian(eef_pos, rv):
    return rv.pdf(eef_pos)

'''
Main
'''
if __name__ == "__main__":
    eef_pos = [0.85, 0.1, -0.11]    
    obj_pos = [0.65, 0.1, -0.11]
    nb_min_orientation_sections = 4
    radius = 0.2
    print_gaussian = True
    
    orien_offset = (2*math.pi/nb_min_orientation_sections)/2
    orien_min_angle = -math.pi + math.pi/2 + orien_offset
    orien_max_angle = math.pi + math.pi/2 + orien_offset    
    list_radians = np.linspace(orien_min_angle, 
                               orien_max_angle, 
                               nb_min_orientation_sections + 1)

    list_x_axis = []
    list_y_axis = []    
    for a in list_radians:
        list_x_axis.append(obj_pos[0] + math.cos(a)*radius)
        list_y_axis.append(obj_pos[1] + math.sin(a)*radius)

    
    rv = gaussian_discretization(obj_pos)    
    check_pos_gaussian(eef_pos, rv)
    
    