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

drone = ps_drone.Drone()       # Initializes the PS-Drone-API

def exit_gracefully(signal, frame):
    print("Shutting down")
    drone.shutdown()

signal.signal(signal.SIGQUIT, exit_gracefully)

drone.startup()                # Connects to the drone and starts subprocesses

drone.takeoff()                # Drone starts
time.sleep(7.5)                # Gives the drone time to start



drone.setSpeed(0.2)            # Sets default moving speed to 1.0 (=100%)
print(drone.setSpeed())         # Shows the default moving speed


drone.land()                   # Drone lands
