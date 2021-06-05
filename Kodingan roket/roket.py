import sys
from i2c_adxl345 import *
from i2c_itg3205 import *
import multiprocessing
from time import sleep
import serial 
from micropyGPS import MicropyGPS
from digi.xbee.devices import XBeeDevice
import math
from bitarray import bitarray

################## Xbee Config ########################################
PORT = "/dev/ttyUSB0"
BAUD_RATE = 57600
REMOTE_NODE_ID = "MAAT2"
DATA_BITS = serial.EIGHTBITS
STOP_BITS=serial.STOPBITS_ONE
PARITY=serial.PARITY_NONE

device = XBeeDevice(PORT, BAUD_RATE, DATA_BITS, STOP_BITS, PARITY)
########################################################################

######################################## GPS Function ####################################################
import time
import re

def dm_to_sd(dm):
    '''
    Converts a geographic co-ordinate given in "degrees/minutes" dddmm.mmmm
    format (eg, "12319.943281" = 123 degrees, 19.943281 minutes) to a signed
    decimal (python float) format
    '''
    # '12319.943281'
    if not dm or dm == '0':
        return 0.
    d, m = re.match(r'^(\d+)(\d\d\.\d+)$', dm).groups()
    return float(d) + float(m) / 60

def latitude(lat,lat_dir):
        '''Latitude in signed degrees (python float)'''
        sd = dm_to_sd(lat)
        if lat_dir == 'N':
            return +sd
        elif lat_dir == 'S':
            return -sd
        else:
            return 0.

def longitude(lon,lon_dir):
    '''Longitude in signed degrees (python float)'''
    sd = dm_to_sd(lon)
    if lon_dir == 'E':
        return +sd
    elif lon_dir == 'W':
        return -sd
    else:
        return 0.
      
def altitude(alt):
    if alt:
        return float(alt)
    else:
        return 0
    
def GGA(line):

    line = line.split(",")
  
    lat_raw = line[2]
    lat_dir = line[3]
    lon_raw = line[4]
    lon_dir = line[5]

    res_lat = latitude(lat_raw,lat_dir)
    res_lon = longitude(lon_raw,lon_dir)
    res_alt = altitude(line[9])

    return res_lat,lat_dir,res_lon,lon_dir,res_alt
#################################################################################################

# Fungsi membaca hasil akselerasi 
def acceleration(result_acc):
    while True:
        (x,y,z) = i2c_adxl345(1).getAxes()
        x=round(x,2)
        y=round(y,2)
        z=round(z,2)

        result_acc.put(x)
        result_acc.put(y)
        result_acc.put(z)

        sleep(0.1)
        

# Fungsi membaca hasil gyro 
def gyro(result_gyro):
    itg3205 = i2c_itg3205(1)
    while True:
        (itgready, dataready) = itg3205.getInterruptStatus()
        if dataready:
            (x, y, z) = itg3205.getDegPerSecAxes()
            x=round(x,2)
            y=round(y,2)
            z=round(z,2)
            # Memasukkan hasil pembacaan ke variabel gyro yang didefine di main.
            result_gyro.put(x)
            result_gyro.put(y)
            result_gyro.put(z)

        sleep(0.1)


# def split_sentence():
#     sentence =  "$GPGGA,12345.07,1929.045,S,02410.506,E,1,04,2.6,100.0,M,-33.9,M,,0000*6D"\
#     "$G$NGGA,11111.07,1929.045,S,02410.506,E,1,04,2.6,100.0,M,-33.9,M,,0000*6D"\
#     "$GPGGA,12222.07,1929.045,S,02410.506,E,1,04,2.6,100.0,M,-33.9,M,,0000*6D"\
#     "$GACCA,555603.07,1929.045,S,02410.506,E,1,04,2.6,100.0,M,-33.9,M,,0000*6D"

#     # 13 commas in a valid input
#     # should return 15 in length
#     valid_data = []
#     pattern = re.compile(r'\$G(N|P)GGA')
#     matches = pattern.finditer(sentence)

#     # "$GPGGA,184353.07,1929.045,S,02410.506,E,1,04,2.6,100.0,M,-33.9,M,,0000*6D"
#     for match in matches:
#         index = match.span()[0] # start index
#         split_data = sentence[index:].split(',')
#         valid_data.append(split_data[index:index+15])
#         return valid_data
    
        
# Fungsi membaca hasil sensor GPS dan mendekode hasil GPS
def gps(result_gps):  
  
    ser = serial.Serial ("/dev/ttyS0",115200,bytesize = serial.EIGHTBITS,
                         stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE )
    while True:
        my_sentence=ser.readline()
        data_formats = [b'$GPGGA', b'$GNGGA']
        
        for format in data_formats:
          if format in my_sentence:
            my_sentence=my_sentence.decode('utf-8')

            lat,lat_dir,lon,lon_dir,alt=GGA(my_sentence)
            res_x=str(round(lat,2))+" "+lat_dir
            res_y=str(round(lon,2))+" "+lon_dir
            z=str(round(alt,2))

            result_gps.put(res_x)
            result_gps.put(res_y)
            result_gps.put(z)
  
        
##########################################

#Kirim data mellui xbee
def send_data(DATA_TO_SEND):
    
    xbee_network = device.get_network()
    remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

    if remote_device is None:
        print("Could not find the remote device")
        exit(1)

    print("Sending data asynchronously to %s >> data :  %s..." % (remote_device.get_64bit_addr(), DATA_TO_SEND))
    device.send_data_async(remote_device, DATA_TO_SEND)
    print("Success")

# Main Thread           
if __name__ == "__main__":
    device.open()
    data=""
    result_gyro=multiprocessing.Queue()
    result_gps=multiprocessing.Queue()
    result_acc=multiprocessing.Queue()
    
    M_PI=3.14159265358979323846
  
    pgyro = multiprocessing.Process(target=gyro, args=(result_gyro, )) #thread pembaca gyro
    pacc = multiprocessing.Process(target=acceleration, args=(result_acc, ))
    pgps = multiprocessing.Process(target=gps, args=(result_gps, )) # thread pembaca gps

    pgyro.start() #mulai thread gyro
    pacc.start()
    pgps.start() # mulai thread fps
    # KElanjutannya, semua thread harus di sinkronisasi, untuk accelerometer harus di ketahui tipe datanya agar bisa disesuaikan di 
    # list acc. Thread pembacaan kamera harus dibuat dan thread prosesing data ke zigbee dibuat di main saja
    print("Starting encoding and transmission")
    while True:
  
        try:
  
            accelerationX = result_acc.get()
            accelerationY = result_acc.get()
            accelerationZ = result_acc.get()
  
            pitch = 180 * math.atan (accelerationX/math.sqrt(accelerationY*accelerationY + accelerationZ*accelerationZ))/M_PI
            roll = 180 * math.atan (accelerationY/math.sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI
            yaw = 180 * math.atan (accelerationZ/math.sqrt(accelerationX*accelerationX + accelerationZ*accelerationZ))/M_PI
     
            data = (str(result_gps.get()) + "," + str(result_gps.get()) + "," + str(result_gps.get()) +
                    "," + str(round(roll,2)) + "," + str(round(pitch,2)) + "," + str(round(yaw,2)) + 
                    "," + str(accelerationX) + "," + str(accelerationY) + "," + str(accelerationZ) +
                    "," + str(result_gyro.get()) + "," + str(result_gyro.get()) + "," + str(result_gyro.get()))
                    
                    
            print(data)
            send_data(data)

            sleep(0.1)
        except KeyboardInterrupt:
            print("Ending Transmission and process")
            pgyro.join()
            pacc.join()
            pgps.join()
            device.close()
            exit(0)