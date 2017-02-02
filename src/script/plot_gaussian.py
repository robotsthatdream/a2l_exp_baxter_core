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
import math, sys


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
def gaussian_discretization(obj_pos,
                            discr_positions):
    
    if print_gaussian:
        # Create a surface plot and projected filled contour plot under it.
        fig = plt.figure()
        ax = fig.gca(projection='3d')

    # Our 2-dimensional distribution will be over variables X and Y
    N = 60
    X = np.linspace(0.45, 0.85, N)
    Y = np.linspace(-0.1, 0.3, N)
    X, Y = np.meshgrid(X, Y)
    ## to make a torus
    discr_positions.append(discr_positions[0])
        
    ## first, compute value for horizontal gaussian in obj pos
    horiz_gaussian = False
    expected_prob_obj_pos = 0
    pos = 0
    while not horiz_gaussian and pos < range(len(discr_positions)-1):
        curr_discr_position = discr_positions[pos]
        next_discr_position = discr_positions[pos+1]
        mid_position = [(curr_discr_position[0] + next_discr_position[0])/2,
                        (curr_discr_position[1] + next_discr_position[1])/2]
                        
        mu = np.array(mid_position)

        ## To compute the orientation of the gaussian
        diff_mid_point = [round(mid_position[0]-obj_pos[0],1),
                          round(mid_position[1]-obj_pos[1],1)]
        
        ## Covariance matrix for horizontal 
        cov1 = 0.005
        cov4 = 0.01
        cov2 = 0             
        cov3 = 0
        if diff_mid_point[0] == 0 : ## same X, horizontal gaussian
            horiz_gaussian = True
            sigma = np.array([[cov1, cov2], [cov3,  cov4]])            
            pos = np.empty(X.shape + (2,))
            pos[:, :, 0] = X
            pos[:, :, 1] = Y            
            rv = multivariate_normal(mean=mu, cov=sigma)
            expected_prob_obj_pos = round(rv.pdf(obj_pos[:-1]), 3)
        pos += 1
    if not horiz_gaussian:
        print('horizontal gaussian not found')        
        sys.exit()
    
    ## then, compute diagonal gaussians with same value is same pos 
    section_vector = []
    layer_pos = 0
    layer_pos_offset = 1
    last_cov1 = -1
    last_cov2 = -1
    last_cov3 = -1
    last_cov4 = -1    
    for pos in range(len(discr_positions)-1):
        
#        if pos==2:
            curr_discr_position = discr_positions[pos]
            next_discr_position = discr_positions[pos+1]
            
            ## Mean vector is the mid pos between the two positions
            ## of each orientation section cutting the radio
            mid_position = [(curr_discr_position[0] + next_discr_position[0])/2,
                            (curr_discr_position[1] + next_discr_position[1])/2]
                            
            mu = np.array(mid_position)

            ## To compute the orientation of the gaussian
            diff_mid_point = [round(mid_position[0]-obj_pos[0],1),
                              round(mid_position[1]-obj_pos[1],1)]
#            print(pos, diff_mid_point)
            
            ## Covariance matrix
            v1 = 0.005
            v2 = 0.01
            cov2 = 0             
            cov3 = 0
            
            if diff_mid_point[0] == 0 : ## same X, horizontal gaussian
                print("\nHorizontal")
                cov1 = v1
                cov4 = v2
                sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                # Pack X and Y into a single 3-dimensional array
                pos = np.empty(X.shape + (2,))
                pos[:, :, 0] = X
                pos[:, :, 1] = Y                
                rv = multivariate_normal(mean=mu, cov=sigma)
                last_cov1 = cov1
                last_cov2 = cov2
                last_cov3 = cov3
                last_cov4 = cov4
            elif diff_mid_point[1] == 0 : ## same Y, vertical gaussian
                print("\nVertical")                
                cov1 = v2
                cov4 = v1
                sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                # Pack X and Y into a single 3-dimensional array
                pos = np.empty(X.shape + (2,))
                pos[:, :, 0] = X
                pos[:, :, 1] = Y                
                rv = multivariate_normal(mean=mu, cov=sigma)                   
                last_cov1 = cov1
                last_cov2 = cov2
                last_cov3 = cov3
                last_cov4 = cov4
            ## always < or >, right-bottom to left-up gaussian
            elif (diff_mid_point[0] > 0 and diff_mid_point[1] > 0) or \
                 (diff_mid_point[0] < 0 and diff_mid_point[1] < 0):
#                cov1 = v2
#                cov2 = v2                     
#                cov3 = v1 - 0.000875
#                cov4 = v1 - 0.000875
#                sigma = np.array([[cov1, cov2], [cov3,  cov4]])
#                pos = np.empty(X.shape + (2,))
#                pos[:, :, 0] = X
#                pos[:, :, 1] = Y
#                rv = multivariate_normal(mean=mu, cov=sigma)    
                
                print("\nDIAGONAL izquierda")
                cov1 = v2
                cov2 = v2
                cov3 = v1
                cov4 = v1
                min_add = 0.00001            
                inclin_found = False
                add_offset = False
                curr_bigger = False
                ## check offset addition or subtraction
                tmp_vector = []
                for i in range(0,2):
                    if i == 0:
                        tmp = cov3 - min_add
                    else:
                        tmp = cov3 + min_add                
                    cov3 = tmp
                    cov4 = tmp                    
                    sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                    pos = np.empty(X.shape + (2,))
                    pos[:, :, 0] = X
                    pos[:, :, 1] = Y    
                    rv = multivariate_normal(mean=mu, cov=sigma)
                    current_prob_obj_pos = round(rv.pdf(obj_pos[:-1]), 3)
                    tmp_vector.append(current_prob_obj_pos)
                    if i == 0:
                        if current_prob_obj_pos > expected_prob_obj_pos:
                            curr_bigger = True
                if curr_bigger: ## we want to decrease number
                    print('CURR BIGGER')
                    if tmp_vector[0] < tmp_vector[1]: ## take smaller
                        add_offset = False
                    else:
                        add_offset = True
                else: ## we want to increase number
                    print('CURR SMALLER')
                    if tmp_vector[0] <= tmp_vector[1]:
                        add_offset = True
                    else:
                        add_offset = False
                    
                ## now compute gaussian orientation
                print('add_offset', add_offset)
                nb_loops = 0
                while not inclin_found and nb_loops < 50:
                    if add_offset:
                        tmp = cov3 + min_add
                    else:
                        tmp = cov3 - min_add
                    cov3 = tmp
                    cov4 = tmp                    
                    sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                    pos = np.empty(X.shape + (2,))
                    pos[:, :, 0] = X
                    pos[:, :, 1] = Y
                    try:
                        rv = multivariate_normal(mean=mu, cov=sigma)
                        current_prob_obj_pos = round(rv.pdf(obj_pos[:-1]), 3)                        
                        diff_prob = abs(expected_prob_obj_pos - current_prob_obj_pos)
                        print(round(cov1, 3), round(cov2, 3), round(cov3, 3), round(cov4, 3)) 
                        print('>>>>>>>', current_prob_obj_pos, diff_prob)
                        if diff_prob <= 0.1: ## stop condition
                            inclin_found = True
                        else:
                            min_add += 0.00001
                    except:
                        min_add += 0.00001
                    nb_loops += 1
                if nb_loops < 50:
                    last_cov1 = cov1
                    last_cov2 = cov2
                    last_cov3 = cov3
                    last_cov4 = cov4
                else:
                    cov1 = last_cov1
                    cov2 = last_cov2
                    cov3 = last_cov3
                    cov4 = last_cov4
                    sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                    pos = np.empty(X.shape + (2,))
                    pos[:, :, 0] = X
                    pos[:, :, 1] = Y
                    rv = multivariate_normal(mean=mu, cov=sigma)                                      
                    
            ## inverse X and Y, left-bottom to right-up gaussian                
            elif (diff_mid_point[0] > 0 and diff_mid_point[1] < 0) or \
                 (diff_mid_point[0] < 0 and diff_mid_point[1] > 0):
#                cov1 = v2
#                cov2 = -v2
#                cov3 = -v1 + 0.000875
#                cov4 = v1 - 0.000875
#                sigma = np.array([[cov1, cov2], [cov3,  cov4]])
#                pos = np.empty(X.shape + (2,))
#                pos[:, :, 0] = X
#                pos[:, :, 1] = Y
#                rv = multivariate_normal(mean=mu, cov=sigma)                
                
                print("\nDIAGONAL derecha")
                cov1 = v2
                cov2 = -v2
                cov3 = -v1
                cov4 = v1
                min_add = 0.00001            
                inclin_found = False
                inclin_found = False
                add_offset = False
                curr_bigger = True
#                print(' ')
                ## check offset addition or subtraction
                tmp_vector = []
                for i in range(0,2):
                    if i == 0:
                        tmp = cov3 + min_add
                        tmp2 = cov4 - min_add
                    else:
                        tmp = cov3 - min_add                
                        tmp2 = cov4 + min_add
                    cov3 = tmp
                    cov4 = tmp2                 
                    sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                    pos = np.empty(X.shape + (2,))
                    pos[:, :, 0] = X
                    pos[:, :, 1] = Y
                    rv = multivariate_normal(mean=mu, cov=sigma)
                    current_prob_obj_pos = round(rv.pdf(obj_pos[:-1]), 3)
                    tmp_vector.append(current_prob_obj_pos)
                    if i == 0:
                        if curr_discr_position > expected_prob_obj_pos:
                            curr_bigger = False
                if curr_bigger: ## we want to decrease number
                    print('CURR BIGGER')
                    if tmp_vector[0] > tmp_vector[1]: ## take smaller
                        add_offset = True
                    else:
                        add_offset = False
                else: ## we want to increase number
                    print('CURR SMALLER')
                    if tmp_vector[0] < tmp_vector[1]:
                        add_offset = True
                    else:
                        add_offset = False
                    
                ## now compute gaussian orientation      
                nb_loops = 0
                while not inclin_found and nb_loops < 50:
                    if add_offset:
                        cov3 += min_add
                        cov4 -= min_add                                            
                    else:
                        cov3 -= min_add
                        cov4 += min_add                                            
                    sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                    pos = np.empty(X.shape + (2,))
                    pos[:, :, 0] = X
                    pos[:, :, 1] = Y
                    try:
                        rv = multivariate_normal(mean=mu, cov=sigma)
                        current_prob_obj_pos = round(rv.pdf(obj_pos[:-1]), 3)
                        print(current_prob_obj_pos)
                        if abs(expected_prob_obj_pos - current_prob_obj_pos) <= 0.2:
                            inclin_found = True
                        else:
                            min_add += 0.00001
                    except:
                        min_add += 0.00001
                    nb_loops += 1
                if nb_loops < 50:
                    last_cov1 = cov1
                    last_cov2 = cov2
                    last_cov3 = cov3
                    last_cov4 = cov4
                else:
                    cov1 = last_cov1
                    cov2 = last_cov2
                    cov3 = last_cov3
                    cov4 = last_cov4
                    sigma = np.array([[cov1, cov2], [cov3,  cov4]])
                    pos = np.empty(X.shape + (2,))
                    pos[:, :, 0] = X
                    pos[:, :, 1] = Y
                    rv = multivariate_normal(mean=mu, cov=sigma)                    
                    
            else:
                print('Error - gaussian_discretization : cov not computed properly')
                sys.exit()
                                
            if print_gaussian:                
                Z = rv.pdf(pos)
                
#                ax.plot_surface(X, Y, Z, rstride=3, cstride=3, 
#                                linewidth=1, antialiased=True,
#                                cmap='Blues')
#                ax.set_zlim(-1,25)
                
                ax.contourf(X, Y, Z, zdir='z', offset=layer_pos, cmap='Blues')
                ax.set_zlim(0,layer_pos_offset*nb_min_orientation_sections)
                
                # Adjust the limits, ticks and view angle
                ax.set_xlim(0.45, 0.85)
                ax.set_ylim(-0.1, 0.3)

                #ax.view_init(27, -21)
                ax.view_init(90, 180) ## robot point of view
                
                ax.set_xlabel('x')
                ax.set_ylabel('y')
                
            layer_pos += layer_pos_offset 
            section_vector.append(rv)                
    plt.show()
    
    return section_vector

'''
Compute distribution probality for a position [x,y]
'''
def check_pos_gaussian(eef_pos, section_vector):
    return [round(rv.pdf(eef_pos), 3) for rv in section_vector]
    
def plot_one_gaussian():
    
    if print_gaussian:
        # Create a surface plot and projected filled contour plot under it.
        fig = plt.figure()
        ax = fig.gca(projection='3d')    
    
    N = 60
    X = np.linspace(-0.2, 0.2, N)
    Y = np.linspace(-0.2, 0.2, N)
    X, Y = np.meshgrid(X, Y)
    ## to make a torus
    discr_positions.append(discr_positions[0])    
    
    pos = 0
    curr_discr_position = [0.1,0] #discr_positions[pos]
    next_discr_position = [-0.1,0] #discr_positions[pos+1]
    
    ## Mean vector is the mid pos between the two positions
    ## of each orientation section cutting the radio
    mid_position = [(curr_discr_position[0] + next_discr_position[0])/2,
                    (curr_discr_position[1] + next_discr_position[1])/2]
                    
    mu = np.array(mid_position)

    ## To compute the orientation of the gaussian
    diff_mid_point = [round(mid_position[0]-obj_pos[0],1),
                      round(mid_position[1]-obj_pos[1],1)]
#            print(pos, diff_mid_point)
    
    ## Covariance matrix for horizontal 
#    v1 = 0.01 horizontal
#    v2 = 0.0
    
    cov1 = 0.01
    cov2 = 0.01
    cov3 = 0.005
    cov4 = 0.005
    sigma = np.array([[cov1, cov2], [cov3,  cov4]])
    pos = np.empty(X.shape + (2,))
    pos[:, :, 0] = X
    pos[:, :, 1] = Y
    rv = multivariate_normal(mean=mu, cov=sigma)
    print(round(rv.pdf(eef_pos), 3))
    
    if print_gaussian:
        Z = rv.pdf(pos)
        
#                ax.plot_surface(X, Y, Z, rstride=3, cstride=3, 
#                                linewidth=1, antialiased=True,
#                                cmap='Blues')
#                ax.set_zlim(-1,25)
        
        ax.contourf(X, Y, Z, zdir='z', offset=0, cmap='Blues')
        ax.set_zlim(0,1)
        
        # Adjust the limits, ticks and view angle
        ax.set_xlim(-0.2, 0.2)
        ax.set_ylim(-0.2, 0.2)

        #ax.view_init(27, -21)
        ax.view_init(90, 180) ## robot point of view
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')    

'''
Main
'''
if __name__ == "__main__":    
    obj_pos = [0.65, 0.1, -0.11]
    nb_min_orientation_sections = 16
    radius = 0.2
    print_gaussian = True
    
    orien_offset = (2*math.pi/nb_min_orientation_sections)/2
#    print('offset', math.degrees(orien_offset))    
    orien_min_angle = -math.pi + orien_offset
    orien_max_angle = math.pi + orien_offset    

    angle = 2*math.pi/nb_min_orientation_sections
    list_radians = [angle*i+orien_offset for i in range(0,nb_min_orientation_sections)]    
#    tmp = [math.degrees(i) for i in list_radians]
#    print(tmp)

    list_x_axis = []
    list_y_axis = []    
    for pos in list_radians:
        list_x_axis.append(obj_pos[0] + math.cos(pos)*radius)
        list_y_axis.append(obj_pos[1] + math.sin(pos)*radius)

    discr_positions = zip(list_x_axis, list_y_axis)
    eef_pos = [0.65, .1]
    rv_vector = gaussian_discretization(obj_pos, discr_positions)    
    
    print(eef_pos)
    print(check_pos_gaussian(eef_pos, rv_vector))
    
#    eef_pos = [0, 0]
#    plot_one_gaussian()
    
    