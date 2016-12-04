# -*- coding: utf-8 -*-
"""
Created on Mon Oct  3 10:53:32 2016

@author: maestre
"""

from __future__ import print_function

from multiprocessing import Pool

def func(init_pos_vector):
    res = []
    for l in init_pos_vector:        
        res.append(l)
    return res


nb_init_pos = 16
init_pos_vector = [i for i in range(nb_init_pos)]

nb_processes = nb_init_pos // 2
pool = Pool(processes = nb_processes)
multiple_results = [pool.apply_async(func, 
                                     ([2*i,2*i+1],)) for i in range(nb_processes)]
#print [res2.get(timeout=10) for res2 in multiple_results]
res = []
for curr_res in multiple_results:
    res += curr_res.get(timeout=10)
print (res)

#result1 = pool.apply_async(func, ['A'])    # evaluate "solve1(A)" asynchronously
#result2 = pool.apply_async(func, ['B'])    # evaluate "solve2(B)" asynchronously
#answer1 = result1.get(timeout=10)
#answer2 = result2.get(timeout=10)

#from multiprocessing import Pool, TimeoutError
#import time
#import os
#
#def f(x):
#    return x*x
#
#if __name__ == '__main__':
#    pool = Pool(processes=4)              # start 4 worker processes
#
#    # print "[0, 1, 4,..., 81]"
#    print pool.map(f, range(10))
#
#    # print same numbers in arbitrary order
#    for i in pool.imap_unordered(f, range(10)):
#        print i
#
#    # evaluate "f(20)" asynchronously
#    res = pool.apply_async(f, (20,))      # runs in *only* one process
#    print res.get(timeout=1)              # prints "400"
#
#    # evaluate "os.getpid()" asynchronously
#    res = pool.apply_async(os.getpid, ()) # runs in *only* one process
#    print res.get(timeout=1)              # prints the PID of that process
#
#    # launching multiple evaluations asynchronously *may* use more processes
#    multiple_results = [pool.apply_async(os.getpid, ()) for i in range(4)]
#    print [res2.get(timeout=1) for res2 in multiple_results]
#
#    # make a single worker sleep for 10 secs
#    res = pool.apply_async(time.sleep, (0,))
#    try:
#        print res.get(timeout=1)
#    except TimeoutError:
#        print "We lacked patience and got a multiprocessing.TimeoutError"