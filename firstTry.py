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
#print(drone.getBattery()[0] == -1: time.sleep(0.1))
time.sleep(0.5)

print("Taking off")
drone.takeoff()                # Drone starts
print("Sleeping")
time.sleep(5)

print("about to move forward")
drone.moveForward()            # Drone flies forward...
time.sleep(1)                  # ... for two seconds
drone.stop()                   # Drone stops...
time.sleep(2)                  # ... needs, like a car, time to stop
print("moved forward")


print("about to move backward")
drone.moveBackward(0.3)       # Drone flies backward with a quarter speed...
time.sleep(1)                # ... for one and a half seconds
drone.stop()                   # Drone stops
time.sleep(2)
print("moved backward")
#
drone.setSpeed(0.4)            # Sets default moving speed to 1.0 (=100%)
print(drone.setSpeed())         # Shows the default moving speed
#

#print("about to turn left")
#drone.turnLeft(0.5)               # Drone moves full speed to the left...
#time.sleep(2)                  # ... for two seconds
#drone.stop()                   # Drone stops
#time.sleep(2)

print("about to turn 90 degrees")
drone.turnAngle(90, 1)        # Begins turning the drone at speed: 1 until the drone has fully turned 90º clockwise, relative to its orientation before calling this function
print("turned 90 degrees")

#print("about to turn right")
#drone.turnRight(0.1)
#time.sleep(0.5)
#drone.stop()
#time.sleep(2)
#print("turned right")

#print("about to turn left")
#drone.turnLeft(0.1)               # Drone moves full speed to the left...
#time.sleep(0.5)                  # ... for two seconds
#drone.stop()                   # Drone stops
#time.sleep(2)
#print("turned left")

print("about to move right")
drone.moveRight(0.1)
time.sleep(1)
drone.stop()
time.sleep(2)
print("moved right")

print("about to move left")
drone.moveLeft(0.5)
time.sleep(1)
drone.stop()
time.sleep(2)
print("moved left")

print("about to move down")
drone.moveDown(0.1)
time.sleep(0.75)
drone.stop()
time.sleep(2)
print("moved down")

print("about to move up")
drone.moveUp(0.3)
time.sleep(1.5)
drone.stop()
time.sleep(2)
print("moved up")


drone.setSpeed(0.1)
print(drone.setSpeed())
#drone.turnLeft(0.1)
#time.sleep(2)
#drone.stop()

print("about to rotate 360 degrees")
time.sleep(2)
drone.turnAngle(360,1)
print("rotated 360 degrees")

print("Landing")
drone.shutdown()                   # Drone lands
print("Done")
