# Example usage of the LSM303DLHC accelerometer/magnetometer

from time import sleep
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math
#import numpy as np

from pynput import keyboard

i2c = busio.I2C(board.SCL, board.SDA)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

end = False

def on_press(key):
    global end
    end = True
    return False

while not end:
    # print("Accelerometer (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
    # print("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f"%mag.magnetic)
    
    accel_x = accel.acceleration[0]
    accel_y = accel.acceleration[1]
    accel_z = accel.acceleration[2]
    
    mag_x = mag.magnetic[0]
    mag_y = mag.magnetic[1]
    mag_z = mag.magnetic[2]
    
    g = 9.8
    
    #make sure to not get an invalid asin value to pass through
    #marks invalid theta by equaling it to 1000
    try:
        theta = math.pi - math.asin(accel_z/g)
        phi = math.atan(mag_y/mag_x)
        print('{},{}'.format(theta,phi))
    except ValueError:
        theta = 1000
        #print(1000)
    # print("theta: ", theta)
    # print("phi: ", phi)
    #print('{},{}'.format(theta,phi))
    #print('%s,%s' %(theta,phi))

    # print(phi)
    break
    #sleep(3)
    