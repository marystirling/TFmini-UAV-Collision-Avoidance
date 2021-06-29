import os
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
# import csv

from time import sleep
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag

i2c = busio.I2C(board.SCL, board.SDA)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

list = []

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')

# converts the data from lidar and accelerometer/magnatometer to Catrtesian coordinates
def calculateCoordinates(dist, theta, phi):
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z = dist * math.sin(theta)
    tuple= (x, y, z)
    list.append(tuple)

# plots Cartesian coordinates in matplotlib 3D scatterplot to visualize point cloud
def plotPoints(list):
    x, y, z = zip(*list)
    ax.scatter(x, y, z, marker='o', s=5)
    plt.savefig('3d_plot.png')
    plt.show()
   

file = open('output.txt', 'w')

i = 100
j=0
while i > 0:
    distanceStr = os.popen('./tfminiTEST.py').read()
    # acc = os.popen('python3 LSM303_TEST.py').read()
    
    g = 9.8
    k = 0
    while k < 1:
        accel_x = accel.acceleration[0]
        accel_y = accel.acceleration[1]
        accel_z = accel.acceleration[2]
        
        mag_x = mag.magnetic[0]
        mag_y = mag.magnetic[1]
        mag_z = mag.magnetic[2]
        
        try:
            theta = math.pi - math.asin(accel_z/g)
            phi = math.atan(mag_y/mag_x)
            #print('{},{}'.format(theta,phi))
        except ValueError:
            theta = 1000
        
        k = k + 1
        break
    
    # only uses the measurements when all three (distance, theta, phi) are valid floats
    if len(distanceStr)>1 and distanceStr != "65535" and theta != "1000":
        distSplit = distanceStr.split("\n")[0]
        distance = float(distSplit)
        
        
        #exception handling for when there is no acc[1] when no value is captured for phi
        print(j)
        j = j + 1
        calculateCoordinates(distance, theta, phi)
  
  
    
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
