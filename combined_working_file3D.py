# imports from ps_drone
import time, sys
import ps_drone                                                   
from time import sleep
import signal

import os
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
# import csv 

# from accelerometer
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

# min distance that the UAV can fly through
marginDist = 20

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')





#currdist = dist
# if(currdist < tooClose):
    #return True
def distTooClose():
    line_edge(coordinateList) 

# compares every element in a list with one another to see if they are in the danger marginDist (where the UAV could not fit through)
def line_edge(coordinateList):
    j = 0
    # iterates through each tuple in the list and compares it to every other tuple in the same list
    # every tuple is compared to all the other tuples in the list
    for aTuple in coordinateList:
        print("comparisons with ", aTuple)
        for otherTuples in coordinateList:
            dist = distBetweenPoints(aTuple, otherTuples)
            print(j)
            j = j + 1
            if dist < marginDist and aTuple != otherTuples:
                print(dist)
                print("too close")
    

# calculates the distance between two tuples (including the coordinates of the front of the UAV if want)
def distBetweenPoints(tuple1, tuple2):
    distance_between_points = math.sqrt(pow(tuple2[0]-tuple1[0],2) + pow(tuple2[1]-tuple1[1],2) + pow(tuple2[2]-tuple1[2],2))
    #print(tuple1, "&", tuple2, ": ", distance_between_points)
    return distance_between_points


# calculates the location of the UAV coordinate by passing in the distance from the lidar to the front of the UAV
def droneLocation(dist, mag_x, mag_y, mag_z):
    theta, phi = accelData(mag_x, mag_y, mag_z)
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

numbers = []
def magData(magnetometerData):
    #print("mag data: ", magnetometerData)
    magnetometerData = magnetometerData[1:-1]
    #print(magnetometerData)
    x, y, z = magnetometerData.split(', ')
    print(x,",",y,",",z)
    #print(x, y, z)
    return x, y, z
    
   #for t in magnetometerData.split():
    #    try:
     #       numbers.append(float(t))
      #      print("mag data pt 2: ", t)
       # except ValueError:
        #    pass


# uses measurements of the acceleration and magnetometers arrays to return the values of theta and phi
def accelData(mag_x, mag_y, mag_z):
    accel_x = accel.acceleration[0]
    accel_y = accel.acceleration[1]
    accel_z = accel.acceleration[2]
    
    mag_x = float(mag_x)
    mag_y = float(mag_y)
    mag_z = float(mag_z)
    
    #mag_x = mag.magnetic[0]
    #mag_y = mag.magnetic[1]
    #mag_z = mag.magnetic[2]
       
    # exception handling to make sure that valid theta and phi arguments are given
    try:
        thetaAcc = math.pi - math.asin(accel_z/g)
        phiAcc = math.atan2(mag_y,mag_x)
        return thetaAcc, phiAcc
            
    except ValueError:
        return 0, 0

# function for emergency drone shutdown (press Ctrl and \ at same time) - must do it in terminal
def exit_gracefully(signal, frame):
    print("Shutting down")
    drone.shutdown()

signal.signal(signal.SIGQUIT, exit_gracefully)




  

print("Initializing")
drone = ps_drone.Drone()       # Initializes the PS-Drone-API
print("Starting")
drone.startup()                # Connects to the drone and starts subprocesses
print("Resetting")
drone.reset()
while drone.getBattery()[0] == -1:	time.sleep(0.1)		# Waits until the drone has done its reset
print ("Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])) # Gives a battery-status
drone.useDemoMode(False)                                                      # Give me everything...fast
drone.getNDpackage(["demo","pressure_raw","altitude","magneto","wifi","wind_speed","euler_angles"])       # Packets, which shall be decoded
time.sleep(0.5)
#time.sleep(1.0)        

#print("Taking off")
#drone.takeoff()                # Drone starts
#print("Sleeping")
#time.sleep(5)


   
   
# i is the number of the data points to capture for point cloud
i = 100
j=0


while i > 0:
    
    # reads from terminal what python2 code is for tf mini
    distanceStr = os.popen('./tfminiTEST.py').read()
    #print ("Megnetometer [X,Y,Z]:         "+str(drone.NavData["magneto"][0]))
    magnetometerData = str(drone.NavData["magneto"][0])
    mag_x, mag_y, mag_z = magData(magnetometerData)
    
    g = 9.8
    k = 0
    while k < 1:
        theta, phi = accelData(mag_x, mag_y, mag_z)
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
dist_drone_to_lidar = 5
magnetometerData = str(drone.NavData["magneto"][0])
mag_x, mag_y, mag_z = magData(magnetometerData)
droneTuple = droneLocation(dist_drone_to_lidar, mag_x, mag_y, mag_z)

# way to iterate through list to compare the distances of it to the front of the UAV
#for i in list:
  #  distBetweenPoints(droneTuple, i)
  
#line_edge(list)

# plots the points by passing in the list of tuples of coordinates
plotPoints(list)




#print("Landing")
#drone.shutdown()                   # Drone lands
#print("Done")