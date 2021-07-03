import math

#file = open('output.txt', 'r')

def remove_newlines(fileName):
    fileName = open(file).readlines()
    return [s.rstrip('\n') for s in fileName]
        
        
def convertCartesian(dist, theta, phi):
    x = dist * math.cos((math.pi/2)-phi)
    y = dist * math.sin((math.pi/2) - phi)
    z - d * math.sin(theta)
    # need to call to store tuple in list
    
def floatConversion(dist_str, theta_str, phi_str):
    print(dist_str)
    print(theta_str)
    print(phi_str)
    dist = float(dist_str)
    theta = float(theta_str)
    phi = float (phi_str)

with open('output.txt', 'r') as file:
    i = 0
    for line in file:
        current_line = line.split(",")
        dist = current_line[0]
        i = i + 1
        theta = current_line[1]
        i = i + 1
        phi = current_line[2]
        i = i + 1          
        
       
        print(dist)
        print(theta)
        print(phi)
        #dist_str = file.readline(i)
        #i = i + 1
        #theta_str = file.readline(i)
        #i + i + 1
        #phi_str = file.readline(i)
        #i = i + 1
        #floatConversion(dist_str, theta_str, phi_str)
        
        #if (dist != 65535):
                #convertCartesian(dist, theta, phi)

        