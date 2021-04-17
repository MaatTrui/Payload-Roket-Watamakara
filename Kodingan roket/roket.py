import sys
sys.path.insert(1,'/home/pi/GY-85_Raspberry-Pi/i2clibraries')
from i2c_adxl345 import *
from i2c_itg3205 import *
import multiprocessing
from time import *
import serial 
from micropyGPS import MicropyGPS

# Fungsi membaca hasil akselerasi 
def acceleration():
    adxl345 = i2c_adxl345(1)
    while True:
        print (adxl345) #perlu dipelajari bentuk datanya gmana
        sleep (1)

# Fungsi membaca hasil gyro 
def gyro():
    itg3205 = i2c_itg3205(2)
    while True:
        (itgready, dataready) = itg3205.getInterruptStatus ()
        if dataready:
            (x, y, z) = itg3205.getDegPerSecAxes ()
            # Memasukkan hasil pembacaan ke variabel gyro yang didefine di main.
            gyro[0] = str(x)
            gyro[1] = str(y)
            gyro[2] = str(z)
    sleep (1))

# Fungsi membaca hasil sensor GPS dan mendekode hasil GPS
def gps():  
    my_gps = MicropyGPS()
    ser = serial.Serial ("/dev/ttyS0",9600,bytesize = serial.EIGHTBITS, stopbits = serial.STOPBITS_ONE, parity = serial.PARITY_NONE)  
    while True:
        my_sentence=ser.readline().decode("utf-8") #decode dulu ke string.   
        if my_sentence.startswith("$GPRMC"): #cek mulai dari gprmc ga (data nmea yang diperlukan)
        for x in my_sentence:
            my_gps.update(x)
            # Memasukkan hasil pembacaan ke variabel gps yang didefine di main.
            gps[0] = my_gps.latitude
            gps[1] = my_gps.longitude
            gps[2] = my_gps.altitude

# Main Thread           
if __name__ == "__main__":
    with multiprocessing.Manager() as manager:
        gyro = manager.list(["0","1","2"]) # Data pembacaan gyroscope disimpan disini
        #acc = manager.list[]
        gps = manager.list(["0","1","2"]) # data pembacaan gps disimpan disini
        pgyro = multiprocessing.Process(target=gyro) #thread pembaca gyro
        #pacc = multiprocessing.Process(target=acceleration)
        pgps = multiprocessing.Process(target=gps) # thread pembaca gps
        pgyro.start() #mulai thread gyro
        #pacc.start()
        pgps.start() # mulai thread fps
        # KElanjutannya, semua thread harus di sinkronisasi, untuk accelerometer harus di ketahui tipe datanya agar bisa disesuaikan di 
        # list acc. Thread pembacaan kamera harus dibuat dan thread prosesing data ke zigbee dibuat di main saja. 
    
