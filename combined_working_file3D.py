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

# creates a coordinates
coordinates = []

# min distance that the UAV can fly through
marginDist = 30

g = 9.8

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')

def createScannedArea(currPos):
    # square_root(2) for distance, 90 degree horizontal and vertical view)
    area = get_search_area(500, 90, 90, currPos)



    #print()
    #print('area to search:', area)
    #print()

    draw_rectangle = [area[0], area[1], area[3], area[2], area[0], currPos, area[1], area[0], currPos, area[3]]
    x_rect = [x for (x, y, z) in draw_rectangle]
    y_rect = [y for (x, y, z) in draw_rectangle]
    z_rect = [z for (x, y, z) in draw_rectangle]

    plt.plot(x_rect, y_rect, z_rect)
    plt.savefig('visualization.jpg')

    #print("drone location: ",currPos)
    for point in coordinates:
        #(point)
    print(search_area(currPos, area, coordinates))

# angleY is left/right angle, z is up/down angle (in degrees)
def get_search_area(distance, angleY, angleZ, position):
    '''returns a list of the four corner coordinates of the projected rectangle 
    based on the current position, horizontal and vertical fields of view, and 
    set distance away'''
    
    # orients from North to East for the polar plane, and converts inputed angle
    # from degrees into radians
    angleY, angleZ = angleY/2, angleZ/2
    angleY, angleZ = 90-angleY, 90-angleZ
    angleY, angleZ = angleY*(math.pi)/180, angleZ*(math.pi)/180
    
    # converts from polar into cartesian
    left  = distance * math.cos(angleY) * -1
    right = distance * math.cos(angleY)
    up    = distance * math.sin(-1*angleZ)
    down  = distance * math.sin(angleZ)
    
    # creates list of the corner points, and orients rectangle to current position
    area = [(-distance, left, up), (-distance, left, down), (-distance, right, up), (-distance, right, down)] 
    area = [(position[0]+point[0], position[1]+point[1], position[2]+point[2] ) for point in area]
    
    # plots points of the projected rectangle for visualization
    for point in area:
        ax.scatter(point[0], point[1], point[2], color='#2ca02c')
    
    return area


def get_plane(a, b, c):
    '''given three coordinates, returns [p, q, r, -d] of px+qy+rz=d plane equation.'''
    ab = (a[0]-b[0], a[1]-b[1], a[2]-b[2])
    ac = (a[0]-c[0], a[1]-c[1], a[2]-c[2])
    cross = list(np.cross(ab, ac))
    cross.append(a[0]*cross[0] + a[1]*cross[1] + a[2]*cross[2])
    return cross


def run_equation(point, plane):
    '''returns the dot product of the normal vector to the plane and a separate point'''
    result = point[0]*plane[0] + point[1]*plane[1] + point[2]*plane[2] - plane[3]
    #print('  ', result)
    return result


def search_area(position, area, space):
    '''given an area to search of the enclosed 3D-shape of the area and points, checks
    each point in the global space area is inside of the space'''
    
    # breaks area space into coordinates - upper left, bottom left, upper right, bottom right
    ul, bl, ur, br = area[0], area[1], area[2], area[3]
    
    # finds the equation of the plane for each side of the enclosed shape
    left = get_plane(position, ul, bl)
    right = get_plane(position, ur, br)
    top = get_plane(position, ul, ur)
    bottom = get_plane(position, bl, br)
    back = get_plane(ul, bl, ur)
    
    # prints the p,q,r data of each plane equation
    #print('planes:')
    #print(left)
    #print(right)
    #print(top)
    #print(bottom)
    #print(back)
    #print()
    
    # checks the points in the space-system to see if inside the enclosed shape
    for point in space:
        #print('Point :', point)
        if (run_equation(point, bottom) > 0):
            continue
        if (run_equation(point, top) < 0):
            continue
        if (run_equation(point, left) > 0):
            continue
        if (run_equation(point, right) < 0):
            continue
        if (run_equation(point, back) < 0):
            continue
        print('the point detected in search area is ', point)
        ax.scatter(point[0], point[1], point[2], color='red', alpha=0.5)
        return True
    
    return False


#currdist = dist
# if(currdist < tooClose):
    #return True
def distTooClose():
    line_edge(coordinatecoordinates) 

# compares every element in a coordinates with one another to see if they are in the danger marginDist (where the UAV could not fit through)
def line_edge(coordinatecoordinates):
    j = 0
    # iterates through each tuple in the coordinates and compares it to every other tuple in the same coordinates
    # every tuple is compared to all the other tuples in the coordinates
    for aTuple in coordinatecoordinates:
        print("comparisons with ", aTuple)
        for otherTuples in coordinatecoordinates:
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
def droneLocation(dist, mag_x, mag_y, mag_z, alt):
    theta, phi = accelData(mag_x, mag_y, mag_z)
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z = dist * math.sin(theta)
    droneTuple = (x, y, alt)
    #print(droneTuple)
    return droneTuple

def droneDataLocation():
    # coordinate of the front of the UAV
    dist_drone_to_lidar = 5
    dist = getDistance()
    #magnetometerData = str(drone.NavData["magneto"][0])
    #mag_x, mag_y, mag_z = magData(magnetometerData)
    mag_x, mag_y, mag_z = getMagData()
    mag_x = float(mag_x)
    mag_y = float(mag_y)
    mag_z = float(mag_z)
    theta, phi = accelData(mag_x, mag_y, mag_z)
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z = dist * math.sin(theta)
    droneAlt = getAltitude()
    currPos = droneLocation(dist_drone_to_lidar, mag_x, mag_y, mag_z, droneAlt)
    print("drone pos ",currPos)
    #droneTuple = (x, y, droneAlt)
    #plotting the current position of the drone
    ax.scatter(currPos[0], currPos[1], currPos[2])
    #print(droneTuple)
    return currPos




# converts the data from lidar and accelerometer/magnatometer to Catrtesian coordinates
def calculateCoordinates(dist, theta, phi, alt):
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z = dist * math.sin(theta)
    tuple = (x, y, alt)
    print(tuple)
    #print(tuple)
    # adds each tuple (aka coordinates) to the coordinates
    coordinates.append(tuple)

# plots Cartesian coordinates in matplotlib 3D scatterplot to visualize point cloud
def plotPoints(coordinates):
    print("plotting points")
    x, y, z = zip(*coordinates)
    ax.scatter(x, y, z, marker='o', s=5)
    plt.savefig('3d_plot.png')
    plt.show()




def getMagData():
    magnetometerData = str(drone.NavData["magneto"][0])
    #mag_x, mag_y, mag_z = magData(magnetometerData)
    #print("mag data: ", magnetometerData)
    magnetometerData = magnetometerData[1:-1]
    #print(magnetometerData)
    x, y, z = magnetometerData.split(', ')
    print(x,",",y,",",z)
    #print(x, y, z)
    return x, y, z
    #return mag_x, mag_y, mag_z

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

def getAltitude():
    altitudeStr = str(drone.NavData["altitude"][3])
    altitude = float(altitudeStr)
    return altitude

def getDistance():
    distanceStr = os.popen('./tfminiTEST.py').read()
    if len(distanceStr)>1 and distanceStr != "65535":
        distSplit = distanceStr.split("\n")[0]
        distance = float(distSplit)
        return distance
    else:
        return "error"

# gets data from scans to get points
# parameter is how many points the lidar scans
def scanPoints(numPoints):
    
    j=0

    #altitudeStr = str(drone.NavData["altitude"][3])
    #altitude = float(altitudeStr)

    while numPoints > 0:
        
        # reads from terminal what python2 code is for tf mini
        #distanceStr = os.popen('./tfminiTEST.py').read()
        #print ("Megnetometer [X,Y,Z]:         "+str(drone.NavData["magneto"][0]))
        #magnetometerData = str(drone.NavData["magneto"][0])
        #mag_x, mag_y, mag_z = magData(magnetometerData)
        mag_x, mag_y, mag_z = getMagData()
        
        altitude = getAltitude()
        distance = getDistance()
       
        
        g = 9.8
        k = 0
        while k < 1:
            theta, phi = accelData(mag_x, mag_y, mag_z)
            k = k + 1
            break
        
        # only uses the measurements when all three (distance, theta, phi) are valid floats
        if distance != "error" and theta != "0" and phi != "0":
            #distSplit = distanceStr.split("\n")[0]
            #distance = float(distSplit)
            
            
            #exception handling for when there is no acc[1] when no value is captured for phi
            print(j)
            j = j + 1
            calculateCoordinates(distance, theta, phi, altitude)
      
        numPoints = numPoints - 1


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

scanPoints(500)
currPos = droneDataLocation()
createScannedArea(currPos)




'''print("about to rotate 360 degrees")
time.sleep(2)
while drone.turnAngle(360,1):
    scanPoints(50)
    currPos = droneDataLocation()
    createScannedArea(currPos)'''
    
    
#print("rotated 360 degrees")




# way to iterate through coordinates to compare the distances of it to the front of the UAV
#for i in coordinates:
  #  distBetweenPoints(droneTuple, i)
  
#line_edge(coordinates)

# plots the points by passing in the coordinates of tuples of coordinates





#print("Landing")
#drone.shutdown()                   # Drone lands
#print("Done")

plotPoints(coordinates)