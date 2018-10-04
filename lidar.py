# Display Data from Neato LIDAR XV-11 
# used with XV Lidar Controller 1.2 https://www.getsurreal.com/product/xv-lidar-controller-v1-2/
# based on code from Nicolas "Xevel" Saugnier


import signal, thread, time, sys, traceback, math, os
import datetime
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing
import serial

com_port = "/dev/cu.usbmodem12341" # This is on mac os x. For Windows/Linux: 5 == "COM6" == "/dev/tty5"
baudrate = 115200

offset = 140
init_level = 0
index = 0

lidarData = [[] for i in range(360)] #A list of 360 elements Angle, Distance , quality


def plotRadial(q):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar' )
    while True:
        if not q.empty():
            a = q.get()
            ax.clear()
            for x in range(0,360):
                if a[x]:
                    theta = 2 * np.pi * x/360.0
                    r = a[x][0]
                    #print x, theta, r
                    ax.scatter(2 * np.pi * x/360.0,a[x][0], cmap='hsv')
            # update ax.viewLim using the new dataLim
            plt.draw()
            plt.pause(0.00000001)
        else:
            print "waiting"
        #time.sleep(0.00001) # do not hog the processor power



def update_view( angle, data ):
    """Updates the view of a sample.

Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
    global offset
    #unpack data using the denomination used during the discussions
    x = data[0]
    x1= data[1]
    x2= data[2]
    x3= data[3]
    
    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)

    dist_mm = x | (( x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8) # quality is on 16 bits
    lidarData[angle] = [dist_mm,quality]
    #print datetime.datetime.now(),angle,dist_mm,quality
    dist_x = dist_mm*c
    dist_y = dist_mm*s
    if angle  >= 359:
        q.put(lidarData)


def checksum(data):
    """Compute and return the checksum as an int.

data -- list of 20 bytes (as ints), in the order they arrived in.
"""
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )


def gui_update_speed(speed_rpm):
    label_speed.text = "RPM : " + str(speed_rpm)

def compute_speed(data):
    speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
    return speed_rpm

def read_Lidar():
    global init_level, angle, index
    
    nb_errors = 0
    while True:
        try:
            time.sleep(0.00001) # do not hog the processor power

            if init_level == 0 :
                b = ord(ser.read(1))
                # start byte
                if b == 0xFA :
                    init_level = 1
                    # print datetime.datetime.now(),lidarData
                    #print "-----------------------------------"
                else:
                    init_level = 0
            elif init_level == 1:
                # position index
                b = ord(ser.read(1))
                if b >= 0xA0 and b <= 0xF9 :
                    index = b - 0xA0
                    init_level = 2
                elif b != 0xFA:
                    init_level = 0
            elif init_level == 2 :
                # speed
                b_speed = [ ord(b) for b in ser.read(2)]
                
                # data
                b_data0 = [ ord(b) for b in ser.read(4)]
                b_data1 = [ ord(b) for b in ser.read(4)]
                b_data2 = [ ord(b) for b in ser.read(4)]
                b_data3 = [ ord(b) for b in ser.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion...
                all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                b_checksum = [ ord(b) for b in ser.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    speed_rpm = compute_speed(b_speed)
                    
                    update_view(index * 4 + 0, b_data0)
                    update_view(index * 4 + 1, b_data1)
                    update_view(index * 4 + 2, b_data2)
                    update_view(index * 4 + 3, b_data3)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1
                    
                    # display the samples in an error state
                    update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    update_view(index * 4 + 3, [0, 0x80, 0, 0])
                    
                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
        except KeyboardInterrupt:
            print ("KeyBoard interrupt")
            job_for_another_core.terminate()
            job_for_another_core.join()
            break
        except Exception:
            traceback.print_exc(file=sys.stdout)

if __name__ == "__main__":

    ser = serial.Serial(com_port, baudrate)
    q = multiprocessing.Queue()
    job_for_another_core = multiprocessing.Process(target=plotRadial,args=((q,)))
    job_for_another_core.start()
    read_Lidar()
    ser.close()
    os._exit(0)
