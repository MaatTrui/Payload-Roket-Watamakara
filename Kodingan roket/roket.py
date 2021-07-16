from i2c_adxl345 import i2c_adxl345
from i2c_itg3205 import i2c_itg3205
import multiprocessing
from time import sleep, gmtime, strftime
from serial import Serial 
from digi.xbee.devices import XBeeDevice
from math import atan2, sqrt, pi
from py_qmc5883l import *
from picamera import PiCamera
from numpy import empty, uint8
import simplejpeg
import re

######################################## Xbee Config #####################################################
PORT = "/dev/ttyUSB0"
ser=Serial(PORT, 57600)
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

                result_gps.put((lat, lon, alt))
        
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
    
            ser.write(data)
            
            camera.stop_preview()
            camera.close()
            print("[+] AMBIL GAMBAR")
            
            queue_signal.get() # empty the queue

        
# Kirim data melalui xbee
'''
def send_data(DATA_TO_SEND):
    xbee_network = device.get_network()
    remote_device = xbee_network.discover_device(REMOTE_NODE_ID)

    if remote_device is None:
        print("Could not find the remote device")
        exit(1)

    print("Sending data asynchronously to %s >> data :  %s..." % (remote_device.get_64bit_addr(), DATA_TO_SEND))
    device.send_data_async(remote_device, DATA_TO_SEND)
    print("Success")
 '''

'''
def data_receive_callback(xbee_message):
    # Instantiate a remote XBee device object.
    xbee_network = device.get_network()
    remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
    #remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A200XXXXXX"))
    
    # Read data sent by the remote device.
    xbee_message = device.read_data(remote_device)
 '''   
# Main Thread           
if __name__ == "__main__":
    # ---------- init variabel ----------
    #Xbee
    '''
    device.open()
    #remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A200XXXXXX"))
    xbee_network = device.get_network()
    remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
    '''
    #
    
    no_data = 0  # data masuk keberapa
    once = True  # default value True
    flight_time = 0.0
    
    result_GY85 = multiprocessing.Queue()
    result_gps = multiprocessing.Queue()
    ground_countrol_signal = multiprocessing.Queue() 
    #ground_countrol_signal.put(1)
    #initial_time=time.time()
    
    print("Starting process")
    
    # init thread
    pGY85 = multiprocessing.Process(target=get_GY85, args=(result_GY85, ground_countrol_signal)) 
    pgps = multiprocessing.Process(target=gps, args=(result_gps, ground_countrol_signal))        
    p_camera = multiprocessing.Process(target=take_picture, args=(ground_countrol_signal, ))     
    
    # mulai thread sensor dan camera
    pGY85.start() 
    #pgps.start() 
    p_camera.start()
    
    # Thread pembacaan kamera harus dibuat dan thread prosesing data ke zigbee dibuat di main saja
    print("Starting encoding and transmission")
    while 1:
        try:
            #xbee_message = device.read_data(remote_device)
            if ser.in_waiting > 0:
                ser.readline()
                ground_countrol_signal.put(1)
            else: 
                acc_data, gyro_data, heading_data = result_GY85.get()
                
                accelerationX, accelerationY, accelerationZ = acc_data
                gyroX, gyroY, gyroZ = gyro_data
                
                roll = 180 * atan(accelerationY/sqrt(accelerationX**2 + accelerationZ**2))/pi
                pitch = 180 * atan(accelerationX/sqrt(accelerationY**2 + accelerationZ**2))/pi
                yaw = 180 * atan(accelerationZ/sqrt(accelerationX**2 + accelerationZ**2))/pi
                
                #lat, lon, alt = result_gps.get()
                lat=0.0;lon=0.0;alt=0.0

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
                """
                data_sensor = [no_data, flight_time, alt, lon, lat, 0.0, 0.0, 0.0, heading_data, roll, pitch, yaw,    
                               accelerationX, accelerationY, accelerationZ, gyroX, gyroY, gyroZ]

                checksum = round(sum(data_sensor), 4)
                data = "$RKT,%s,%s" % (','.join(map(str, data_sensor)), checksum)
                #data = "$RKT,%s,%s\r\n" % (','.join(map(str, data_sensor)), checksum) # Yang ini kah?
                data = "%s%s\n\r" % (data, len(data)) # sisipkan world length ke data
                """
                # kalau ada sinyal dari ground control stop sending data sensor
                
                    #device.send_data_async(remote_device, data)
                #send_data(data)                                  
                print(data)
                ser.write(str.encode(data))
                no_data += 1
            # pseudo ground control station
            '''
            if (xbee_message == 'cam'):
                # imitate a signal
                ground_countrol_signal.put(1)
            '''
      
            sleep(0.1)
        
            
        except KeyboardInterrupt:
            print("\nEnding Transmission and process")
            pGY85.join()
            pgps.join()
            p_camera.join()
            #device.close()
            exit(0)
