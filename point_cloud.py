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

# creates a list
list = []

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')

# calculates the distance between two tuples (including the coordinates of the front of the UAV if want)
def distBetweenPoints(tuple1, tuple2):
    distance_between_points = math.sqrt(pow(tuple2[0]-tuple1[0],2) + pow(tuple2[1]-tuple1[1],2) + pow(tuple2[2]-tuple1[2],2))
    #print(tuple1, "&", tuple2, ": ", distance_between_points)
    return distance_between_points


# calculates the location of the UAV coordinate by passing in the distance from the lidar to the front of the UAV
def droneLocation(dist):
    theta, phi = accelData()
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z = dist * math.sin(theta)
    droneTuple = (x, y, z)
    #print(droneTuple)
    return droneTuple

# converts the data from lidar and accelerometer/magnatometer to Catrtesian coordinates
def calculateCoordinates(dist, theta, phi):
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z = dist * math.sin(theta)
    tuple = (x, y, z)
    #print(tuple)
    # adds each tuple (aka coordinates) to the list
    list.append(tuple)

# plots Cartesian coordinates in matplotlib 3D scatterplot to visualize point cloud
def plotPoints(list):
    x, y, z = zip(*list)
    ax.scatter(x, y, z, marker='o', s=5)
    plt.savefig('3d_plot.png')
    plt.show()

# uses measurements of the acceleration and magnetometers arrays to return the values of theta and phi
def accelData():
    accel_x = accel.acceleration[0]
    accel_y = accel.acceleration[1]
    accel_z = accel.acceleration[2]
    
    mag_x = mag.magnetic[0]
    mag_y = mag.magnetic[1]
    mag_z = mag.magnetic[2]
       
    # exception handling to make sure that valid theta and phi arguments are given
    try:
        thetaAcc = math.pi - math.asin(accel_z/g)
        #phiAcc = math.atan(mag_y/mag_x)
        phiAcc = math.atan2(mag_y, mag_x)
        return thetaAcc, phiAcc
            
    except ValueError:
        return 0, 0
    
    except ZeroDivisionError:
        return 0, 0
   
   
# i is the number of the data points to capture for point cloud
i = 500
j=0


while i > 0:
    
    # reads from terminal what python2 code is for tf mini
    distanceStr = os.popen('./tfminiTEST.py').read()

    
    g = 9.8
    k = 0
    while k < 1:
        theta, phi = accelData()
        k = k + 1
        break
    
    # only uses the measurements when all three (distance, theta, phi) are valid floats
    if len(distanceStr)>1 and distanceStr != "65535" and theta != "0" and phi != "0":
        distSplit = distanceStr.split("\n")[0]
        distance = float(distSplit)
        
        
        #exception handling for when there is no acc[1] when no value is captured for phi
        print(j)
        j = j + 1
        calculateCoordinates(distance, theta, phi)
  
    i = i - 1
    
# coordinate of the front of the UAV
droneTuple = droneLocation(5)

# way to iterate through list to compare the distances of it to the front of the UAV
for i in list:
    distBetweenPoints(droneTuple, i)

# plots the points by passing in the list of tuples of coordinates
plotPoints(list)
                     
