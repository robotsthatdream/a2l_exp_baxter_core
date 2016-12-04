# -*- coding: utf-8 -*-
"""
Created on Fri Oct 21 11:35:03 2016

@author: maestre
"""

#from subprocess import check_call, check_output
import subprocess
import multiprocessing
from time import sleep

#def run_roslaunch():
##    call('./baxter.sh')
##    Popen('"roslaunch /home/maestre/indigo/baxter_ws/src/adaptive_affordance_learning_actions/launch/initialize_experiments.launch"')
#    check_output([
#        'roslaunch',
#        '/home/maestre/indigo/baxter_ws/src/adaptive_affordance_learning_actions/launch/initialize_experiments.launch'])
#
#def test():
#    for i in range(30):
#        print(i)
#    sleep(.1)
#
#p = multiprocessing.Process(name='p', 
#                            target=run_roslaunch)
#p2 = multiprocessing.Process(name='p2', 
#                            target=test)                            
#p.run()
#p2.run()
#  
#p.terminate()
#p2.terminate()
#

mycommandline = ['roslaunch', 
                 '/home/maestre/indigo/baxter_ws/src/adaptive_affordance_learning_actions/launch/initialize_experiments.launch']
subprocess.call(mycommandline)