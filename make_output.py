import os
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
import csv

list = []

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')


def calculateCoordinates(dist, theta, phi):
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z = dist * math.sin(theta)
    tuple= (x, y, z)
    list.append(tuple)
    
def plotPoints(list):
    #plt.scatter(*zip(*list))
    x, y, z = zip(*list)
    ax.scatter(x, y, z, marker='o', s=5)
    plt.show()
    plt.savefig('3d_plot.png')
   

file = open('output.txt', 'w')

i = 250
j=0
while i > 0:
    distanceStr = os.popen('./tfminiTEST.py').read()
    acc = os.popen('python3 LSM303_TEST.py').read()
    
    
    if len(distanceStr)>1 and distanceStr != "65535" and acc != "1000":
        distSplit = distanceStr.split("\n")[0]
        distance = float(distSplit)
        
        
        #exception handling for when there is no acc[1] when no value is captured for phi
        try:
            acc = acc.strip().split(",")
            thetaAcc = acc[0]
            phiAcc = acc[1]
            theta = float(thetaAcc)
            phi = float(phiAcc)
            j = j +1
            print(j)
            calculateCoordinates(distance, theta, phi)
        except IndexError:
            phiAcc = 'null'
            print("exception caught")
    
    # print(distance, 'distance')
    # print(acc, 'accelerometer')
    #print(distanceStr.split("\n")[0]+","+acc)
    #print(acc)
    #file.write('%s,%s\n' %(distanceStr.split("\n")[0],acc))
    #file.write('{},{}'.format(distance,acc))
    # file.write(acc)
    
    i = i - 1
    


plotPoints(list)

file.close()                         