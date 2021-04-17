import serial 
import time
from micropyGPS import MicropyGPS
my_gps = MicropyGPS()
ser = serial.Serial ("/dev/ttyS0",9600,bytesize = EIGHTBITS)  
while True:
    my_sentence=ser.readline()  
    for x in my_sentence:
        my_gps.update(x)
        print(my_gps.latitude)  
        
    