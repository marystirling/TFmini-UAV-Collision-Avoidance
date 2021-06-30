"""
Demo the Bebop indoors (sets small speeds and then flies just a small amount)
Note, the bebop will hurt your furniture if it hits it.  Even though this is a very small
amount of flying, be sure you are doing this in an open area and are prepared to catch!

Author: Amy McGovern
"""

from pyparrot.Bebop import Bebop


#def myFlight(option):
 #   landing = "land"
  #  fly = "fly"
   # if(option == landing):
    #    print("bebop.emergency_landing")
   # elif(option == fly):
        #some code that does nothing
    #    print("NULL")
    
    

#flightChoice = input("What would you like to do: ")
#flightChoiceChecker = myFlight(flightChoice)


# creating the bebop object
bebop = Bebop(drone_type="Bebop2")



# connects to drone with connect(num_retries)
print("connecting")
success = bebop.connect(10)
print(success)

if (success):

    print("sleeping")
    bebop.smart_sleep(2)

    #bebop.ask_for_state_update()
    
    
    # checks sensors to ensure bebop is taking off. returns when bebop is flying/hovering
    # will timeout if exceeds timeout seconds
    bebop.safe_takeoff(10)
    
    bebop.set_max_altitude(0.5)
    
    bebop.emergency_land()

    # set safe indoor parameters
    # set max tilt in degrees for drone between 5 (very slow) and 30 (really fast) degrees
    bebop.set_max_tilt(5)
    # set max vertical speed in m/s (between 0.5 and 2.5)
    bebop.set_max_vertical_speed(0.5)
    
    bebop.set_max_distance(10)

    
    # fly directly with different movements for duration of number of seconds
    # percentage & directoin of the max_tilt (for roll/pitch) or max_vertical_speed (vertical movement)
    #print("Flying direct: Slow move for indoors")
    #bebop.fly_direct(roll=0, pitch=20, yaw=0, vertical_movement=0, duration=2)

    bebop.smart_sleep(5)
    
    # sends command until bebop has actually reached landed state. timeout if exceed timeout seconds
    bebop.safe_land(10)

    #print("DONE - disconnecting")
    
    # sleeps for number of seconds but wakes for wifi notifications
    bebop.smart_sleep(5)
    print(bebop.sensors.battery)
    
    #disconnect from bebop wifi connection
    bebop.disconnect()