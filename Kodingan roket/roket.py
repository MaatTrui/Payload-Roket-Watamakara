import sys
from i2c_adxl345 import *
from i2c_itg3205 import *
import multiprocessing
from time import sleep, time

import serial 
from micropyGPS import MicropyGPS
from digi.xbee.devices import XBeeDevice
from math import atan, sqrt
from bitarray import bitarray
from py_qmc5883l import *
from picamera import PiCamera
import numpy as np
import simplejpeg

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
def get_acceleration(acc_object):
    (x,y,z) = acc_object.getAxes()
    
    return (x, y, z)

# Fungsi membaca hasil gyro 
def get_gyro(gyro_object):
    (itgready, dataready) = gyro_object.getInterruptStatus()
    if dataready:
        (x, y, z) = gyro_object.getDegPerSecAxes()
        
    return (x, y, z)

# Fungsi membaca hasil magnetometer
def get_heading(magnet_object):
    x, y = magnet_object.get_magnet()[:2]
    PI = math.pi
    
    heading = math.atan2(y, x) # raw
    # perlu ditambahin sudut deklinasi mangetik supaya lebih akurat 
    # dan konfigurasi setting magnetometer

    return heading * 180/PI
      
# Fungsi membaca sensor GY-85
def get_GY85(result_GY85, queue_signal):
    adxl345 = i2c_adxl345(1)
    itg3205 = i2c_itg3205(1)
    qmc5883l = QMC5883L()

    while True:
        if (queue_signal.empty()):
            acc_coord = get_acceleration(adxl345)
            gyro_coord = get_gyro(itg3205)
            heading = get_heading(qmc5883l)
            
            result_GY85.put((acc_coord, gyro_coord, heading))
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
def gps(result_gps, queue_signal):  
    ser = serial.Serial ("/dev/ttyS0",115200,bytesize = serial.EIGHTBITS,
                         stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE )
    while True:
        if (queue_signal.empty()):
            my_sentence=ser.readline()
            data_formats = [b'$GPGGA', b'$GNGGA']
            
            for format in data_formats:
              if format in my_sentence:
                my_sentence=my_sentence.decode('utf-8')

                lat,lat_dir,lon,lon_dir,alt=GGA(my_sentence)

                res_x = "%.4f" % (lat) 
                res_y = "%.4f" % (lon) 
                z = "%.2f" % alt 

                result_gps.put(res_x)
                result_gps.put(res_y)
                result_gps.put(z)
  
         
##########################################
# Fungsi ambil gambar, kompres dan kirim 
def take_picture(queue_signal):
    while True:
        # if receive a signal then start the camera
        if (not queue_signal.empty()):
            print("ignore sensor")
            print("start camera")
            camera=PiCamera()
            camera.resolution = (1280,720)
            camera.start_preview()
            
            sleep(2)
            
            output = np.empty((720, 1280, 3), dtype=np.uint8)
            camera.capture(output, 'rgb')
    
            data = simplejpeg.encode_jpeg(output, 90, "RGB", "444") # return bytes
            # -- kode buat send xbee ---
            
            camera.stop_preview()
            camera.close()
            print("TAKE GAMBAR")
            #send(ouput)
            queue_signal.get() # empty the queue

        
# Kirim data melalui xbee
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
    #device.open()
    i = 0 # data masuk keberapa
    once = 0
    flight=0.0
    result_GY85 = multiprocessing.Queue()
    result_gps = multiprocessing.Queue()
    ground_countrol_signal = multiprocessing.Queue() 
    ground_countrol_signal.put(1)
    #initial_time=time.time()
    
    M_PI=3.14159265358979323846
    
    print("Starting process")
    
    pGY85 = multiprocessing.Process(target=get_GY85, args=(result_GY85, ground_countrol_signal)) # thread pembaca GY85
    pgps = multiprocessing.Process(target=gps, args=(result_gps, ground_countrol_signal))        # thread pembaca gps
    p_camera = multiprocessing.Process(target=take_picture, args=(ground_countrol_signal, ))     
    
    pGY85.start() # mulai thread GY85
    pgps.start() # mulai thread gps
    p_camera.start()
    
    # kelanjutannya, semua thread harus di sinkronisasi, untuk accelerometer harus di ketahui tipe datanya agar bisa disesuaikan di 
    # list acc. Thread pembacaan kamera harus dibuat dan thread prosesing data ke zigbee dibuat di main saja
    print("Starting encoding and transmission")
    while True:
        try:
            acc_data, gyro_data, heading_data = result_GY85.get()
            accelerationX, accelerationY, accelerationZ = acc_data
            gyroX, gyroY, gyroZ = gyro_data
            
            roll = 180 * atan(accelerationY/sqrt(accelerationX**2 + accelerationZ**2))/M_PI
            pitch = 180 * atan(accelerationX/sqrt(accelerationY**2 + accelerationZ**2))/M_PI
            yaw = 180 * atan(accelerationZ/sqrt(accelerationX**2 + accelerationZ**2))/M_PI
            
            #lat = result_gps.get()
            #lon = result_gps.get()
            #alt = result_gps.get()
            
            if (once == 0 and sqrt(pow(accelerationX,2)+pow(accelerationY,2)+pow(accelerationZ,2))>1,25):
                initial_time=time.time()
                once==1
            
            if (once == 1):
                flight=time.strftime("%M.%S", time.gmtime(time.time() - initial_time))
            
            # data sensor, alt,lon dan lat 4 decimal point
            #arr_datas = [flight, alt, lon, lat, 0, 0, 0, heading_data, roll, pitch, yaw, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ]         
            arr_datas = [0.0, 0.0, 0.0, heading_data, roll, pitch, yaw, accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ] 
            
            # data = index + sensor + checksum
            data = "%s,%.4f" % (",".join([f"{i:.2f}" for i in arr_datas]), sum(arr_datas))
            data = f"$RKT,{i},{flight},{data},{len(data)},<CR><LF>"
            
            print(data)
            #send_data(data)
            
            """
            # pseudo code for receiving signal from ground control
            if (signal_is_not_empty): 
                queue_signal.append(signal) # thread will call the camera function
                                            # and pop the value for the next incoming signal
            """
            # pseudo ground control station
            #if (ground_countrol_signal.empty() == True):
                # imitate a signal
            #    ground_countrol_signal.put(1)
            
            i += 1
            sleep(0.1)
        except KeyboardInterrupt:
            print("Ending Transmission and process")
            pGY85.join()
            pgps.join()
            #device.close()
            exit(0)
