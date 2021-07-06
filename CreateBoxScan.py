#########
# firstTry.py
# This program is part of the online PS-Drone-API-tutorial on www.playsheep.de/drone.
# It shows how to do basic movements with a Parrot AR.Drone 2.0 using the PS-Drone-API.
# Dependencies: a POSIX OS, PS-Drone-API 2.0 beta or higher.
# (w) J. Philipp de Graaff, www.playsheep.de, 2014
##########
# LICENCE:
#   Artistic License 2.0 as seen on http://opensource.org/licenses/artistic-license-2.0 (retrieved December 2014)
#   Visit www.playsheep.de/drone or see the PS-Drone-API-documentation for an abstract from the Artistic License 2.0.
###########

import time
import ps_drone                # Imports the PS-Drone-API
import sys
import signal

 
def moveRight():
    drone.moveRight(0.2)
    time.sleep(0.5)
    drone.stop()
    time.sleep(2)
    
def moveLeft():
    drone.moveLeft(0.2)
    time.sleep(0.5)
    drone.stop()
    time.sleep(2)

def moveUp():
    drone.moveUp(0.5)
    time.sleep(0.4)
    drone.stop()
    time.sleep(2)

def moveDown():
    drone.moveDown(0.3)
    time.sleep(0.4)
    drone.stop()
    time.sleep(2)

def droneCompleteSquare():
    moveRight()
    moveUp()
    moveLeft()
    moveLeft()
    moveDown()
    moveRight()
    moveDown()
    moveRight()
    moveUp()
    moveLeft()
    moveDown()
    moveLeft()
    moveUp()
    moveRigh()

def movementToCompleteSquare():
    moveRight()
    moveLeft()
    moveLeft()
    moveUp()
    moveRight()
    moveRight()
    moveUp()
    moveLeft()
    moveLeft()
    moveDown()
    moveDown()
    moveDown()
    moveRight()
    moveRight()
    moveDown()
    moveLeft()
    moveLeft()
    moveUp()
    moveUp()
    moveRight()


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
time.sleep(0.5)

print("Taking off")
drone.takeoff()                # Drone starts
print("Sleeping")


#movementToCompleteSquare()
droneCompleteSquare()

print("Landing")
drone.shutdown()                   # Drone lands
print("Done")
