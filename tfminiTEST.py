#!/usr/bin/env python2.7
# -*- coding: utf-8 -*
import serial
import time
ser = serial.Serial("/dev/ttyAMA0", 115200)

def getTFminiData():
    #time1 = time.time()
    while True:
        count = ser.in_waiting
        if count > 8:
            recv = ser.read(9)
            # print("serial in waiting larger than 8")
            ser.reset_input_buffer()
            if recv[0] == 'Y' and recv[1] == 'Y': # 0x59 is 'Y'
                # print('Both first and second index recieved')
                low = int(recv[2].encode('hex'), 16)
                high = int(recv[3].encode('hex'), 16)
                distance = low + high * 256
                if (distance != 65535):
                    print(distance)
                    #print(time1-time.time(), 'seconds')
                    break
            else:
                #print("Critical failure reached, receive failed!")
                break


if __name__ == '__main__':
    try:
        if ser.is_open == False:
            ser.open()
        getTFminiData()
        # print("main_if_statement achieved")
    except KeyboardInterrupt:   # Ctrl+C
        if ser != None:
            ser.close()
