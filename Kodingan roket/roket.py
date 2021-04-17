import sys
from i2c_adxl345 import *
from i2c_itg3205 import *
import multiprocessing
from time import sleep
import serial 
from micropyGPS import MicropyGPS

#gyro_data = ["0","1","2"] # Data pembacaan gyroscope disimpan disini
#acc = []
#gps_data = ["0","1","2"] # data pembacaan gps disimpan disini
# Fungsi membaca hasil akselerasi 
def acceleration(result_acc):
    (x,y,z) = i2c_adxl345(1).getAxes()
    while True:
        result_acc[0]=x
        result_acc[1]=y
        result_acc[2]=z
        """print(f"Accel X = {x}")
        print(f"Accel Y = {y}")
        print(f"Accel Z = {z}")"""
        sleep(1)

# Fungsi membaca hasil gyro 
def gyro(result_gyro):
    itg3205 = i2c_itg3205(1)

    while True:
        (itgready, dataready) = itg3205.getInterruptStatus()
        if dataready:
            (x, y, z) = itg3205.getDegPerSecAxes()
            # Memasukkan hasil pembacaan ke variabel gyro yang didefine di main.
            """
            gyro_data[0] = str(x)
            gyro_data[1] = str(y)
            gyro_data[2] = str(z)
            """
            result_gyro[0] = x
            result_gyro[1] = y
            result_gyro[2] = z
            
            #print(gyro_data[0])
    sleep(1)

# Fungsi membaca hasil sensor GPS dan mendekode hasil GPS
def gps(result_gps):  
    my_gps = MicropyGPS()

    ser = serial.Serial ("/dev/ttyS0",9600,bytesize = serial.EIGHTBITS,
                         stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE )
    my_sentence=ser.readline()
    sleep(10)
    while True:
        my_sentence=ser.readline()
        if my_sentence.startswith(b"$GNGGA", 0, 7):
            for x  in my_sentence.decode('utf-8'):
                my_gps.update(x)
                """
                gps_data[0] = my_gps.latitude
                gps_data[1] = my_gps.longitude
                gps_data[2] = my_gps.altitude
                """
                result_gps[0] = my_gps.latitude
                result_gps[1] = my_gps.longitude
                result_gps[2] = my_gps.altitude
        sleep(1)

# Main Thread           
if __name__ == "__main__":
    result_gyro = multiprocessing.Array('f', 3)
    result_gps = multiprocessing.Array('f', 3)
    result_acc = multiprocessing.Array('f', 3)
    
    pgyro = multiprocessing.Process(target=gyro, args=(result_gyro, )) #thread pembaca gyro
    
    pacc = multiprocessing.Process(target=acceleration, args=(result_acc, ))
    
    pgps = multiprocessing.Process(target=gps, args=(result_gps, )) # thread pembaca gps
    
    pgyro.start() #mulai thread gyro
    
    pacc.start()
    
    pgps.start() # mulai thread fps
    # KElanjutannya, semua thread harus di sinkronisasi, untuk accelerometer harus di ketahui tipe datanya agar bisa disesuaikan di 
    # list acc. Thread pembacaan kamera harus dibuat dan thread prosesing data ke zigbee dibuat di main saja.
    
    
    while True:
        print(f"Gyro x = {result_gyro[0]}")
        print(f"Gyro y = {result_gyro[1]}")
        print(f"Gyro z = {result_gyro[2]}")
        print(f"GPS Latitude = {result_gps[0]}")
        print(f"GPS Longitude = {result_gps[1]}")
        print(f"GPS Altitude = {result_gps[2]}")
        print(f"Accel x = {result_acc[0]}")
        print(f"Accel y = {result_acc[1]}")
        print(f"Accel z = {result_acc[2]}\n")
        
        sleep(1)

        

    
