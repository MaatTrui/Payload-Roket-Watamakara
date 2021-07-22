from i2c_adxl345 import i2c_adxl345
from i2c_itg3205 import i2c_itg3205
import multiprocessing
from time import sleep, gmtime, strftime
from serial import Serial, EIGHTBITS, STOPBITS_ONE, PARITY_NONE
from digi.xbee.devices import XBeeDevice
from math import atan2, sqrt, pi
from py_qmc5883l import *
from picamera import PiCamera
from numpy import empty, uint8
import simplejpeg
import re

######################################## Xbee Config #####################################################

#device_xbee=Serial('/dev/ttyUSB1', 57600)

##########################################################################################################

######################################## GPS Function ####################################################
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

######################################## Sensors Function ####################################################
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
    
    heading = atan2(y, x) # raw
    # perlu ditambahin sudut deklinasi mangetik supaya lebih akurat 
    # dan konfigurasi setting magnetometer

    heading = heading * 180/pi
    
    return heading
      
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

# Fungsi membaca hasil sensor GPS dan mendekode hasil GPS
def gps(result_gps, queue_signal,data_queue):  
    ser = Serial("/dev/ttyS0",57600,bytesize = EIGHTBITS,stopbits = STOPBITS_ONE, parity = PARITY_NONE)
    device_xbee=Serial('/dev/ttyUSB0', 57600)
    while True:
        if (queue_signal.empty() and data_queue.empty()):
            #ser = Serial("/dev/ttyS0",57600,bytesize = EIGHTBITS, stopbits = STOPBITS_ONE, parity = PARITY_NONE )
            my_sentence=ser.readline()
            print(my_sentence)
            #print(my_sentence.decode())
            
            data_formats = [b'$GPGGA']
            
            for format in data_formats:
                if format in my_sentence:
                    try:
                        print(my_sentence)
                        my_sentence=my_sentence.decode('utf-8')
                        lat,lat_dir,lon,lon_dir,alt=GGA(my_sentence)

                        result_gps.put((lat, lon, alt))
                    except UnicodeDecodeError:
                        pass
        else:
            #device_xbee=Serial('/dev/ttyUSB0', 57600)
            device_xbee.write(str.encode(data_queue.get()))
            print('data sent')
            #data_queue.get()
            #ser = Serial("/dev/ttyS0",57600,bytesize = EIGHTBITS,stopbits = STOPBITS_ONE, parity = PARITY_NONE)
            
            sleep(0.1)          
        
##############################################################################################################
# Fungsi ambil gambar, kompres dan kirim 
def take_picture(queue_signal):
    while True:
        # if receive a signal then start the camera
        if (not queue_signal.empty()):

            print("[*] Ignore sensor")
            print("[+] start camera")
            camera=PiCamera()
            camera.resolution = (1280,720)
            camera.start_preview()
            
            sleep(2)

            # bikin array numpy
            output = empty((720, 1280, 3), dtype=uint8)
            camera.capture(output, 'rgb')

            # encode 
            data = simplejpeg.encode_jpeg(output, 90, "RGB", "444") # return bytes
    
            device_xbee.write(data)
            
            camera.stop_preview()
            camera.close()
            print("[+] AMBIL GAMBAR")
            
            queue_signal.get() # empty the queue

def send_data(data_queue,l):
    while True:
        if (not data_queue.empty()):
            l.acquire()
            try:
                print("sending data")
                device_xbee.write(str.encode(data_queue.get()))
            finally:
                l.release()

def receive_data(queue_signal):
    #xbee_message = device.read_data(remote_device)
    while True:
        if device_xbee.in_waiting > 0:
            queue_signal.put(1)

# Main Thread           
if __name__ == "__main__":
    # ---------- init variabel ----------
    lock = multiprocessing.Lock()

    no_data = 0  # data masuk keberapa
    once = True  # default value True
    flight_time = 0.0
    
    result_GY85 = multiprocessing.Queue()
    result_gps = multiprocessing.Queue()
    ground_control_signal = multiprocessing.Queue() 
    data_queue = multiprocessing.Queue()

    
    print("Starting process")
    
    # init thread
    pGY85 = multiprocessing.Process(target=get_GY85, args=(result_GY85, ground_control_signal)) 
    pgps = multiprocessing.Process(target=gps, args=(result_gps, ground_control_signal,data_queue))        
    p_camera = multiprocessing.Process(target=take_picture, args=(ground_control_signal, ))
    #p_send = multiprocessing.Process(target=send_data,args=(data_queue,lock))  
    #p_receive = multiprocessing.Process(target = receive_data, args = (ground_control_signal,))
    
    # mulai thread sensor dan camera
    pGY85.start() 
    pgps.start() 
    p_camera.start()
    #p_send.start()
    #p_receive.start()
    
    # Thread pembacaan kamera harus dibuat dan thread prosesing data ke zigbee dibuat di main saja
    print("Starting encoding and transmission")
    while 1:
        try:
            #xbee_message = device.read_data(remote_device)
            #if device_xbee.in_waiting > 0:
            '''
            if (False):
                device_xbee.readline()
                ground_countrol_signal.put(1)
            '''
            
            acc_data, gyro_data, heading_data = result_GY85.get()
            
            accelerationX, accelerationY, accelerationZ = acc_data
            gyroX, gyroY, gyroZ = gyro_data
            
            roll  = atan2(accelerationY, accelerationZ)*57.3
            pitch = atan2((-accelerationX) , sqrt(accelerationY * accelerationY + accelerationZ * accelerationZ)) * 57.3;
            yaw = atan2(accelerationY, accelerationX)*57.3
            #roll = 180 * atan2(accelerationY/sqrt(accelerationX**2 + accelerationZ**2))/pi
            #pitch = 180 * atan2(accelerationX/sqrt(accelerationY**2 + accelerationZ**2))/pi
            #yaw = 180 * atan2(accelerationZ/sqrt(accelerationX**2 + accelerationZ**2))/pi
            
            lat, lon, alt = result_gps.get()
            #lat, lon, alt 
            #lat=0.0;lon=0.0;alt=0.0

            # jika sudah melebihi 1.25G mulai record flight time
            if (once and sqrt(pow(accelerationX,2)+pow(accelerationY,2)+pow(accelerationZ,2))>1.25):
                initial_time = time.time()
                once = False
            
            if (not once):
                flight_time = float(strftime("%M.%S", gmtime(time.time() - initial_time)))

            # ------ format data ------
            checksum = no_data+flight_time+alt+lon+lat+0.0+0.0+0.0+heading_data+roll+pitch+yaw+accelerationX+accelerationY+accelerationZ+gyroX+gyroY+gyroZ
            data = "$RKT,%d,%s,%0.2f,%0.4f,%0.4f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f"%(no_data,flight_time,alt,lon,lat,0.0,0.0,0.0,heading_data,roll,pitch,yaw,accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ,checksum)
            
            # data = "%s,%.4f,%s<CR><LN>" % (data, checksum, len(data)+str(len(checksum)) + 2 + 8) #Cek footernya diitung berapa karakter
            data = "%s,%s\r\n" % (data, len(data) + 2 + 2) # len + koma + footer
            
            print(data)
            data_queue.put(data)
            
            #send_data(data)                                  
            no_data += 1
            # pseudo ground control station
            '''
            if (xbee_message == 'cam'):
                # imitate a signal
                ground_countrol_signal.put(1)
            '''
            
            #print("A")
      
            sleep(0.1)

        except KeyboardInterrupt:
            print("\nEnding Transmission and process")
            pGY85.join()
            pgps.join()
            p_camera.join()
            #device.close()
            exit(0)
