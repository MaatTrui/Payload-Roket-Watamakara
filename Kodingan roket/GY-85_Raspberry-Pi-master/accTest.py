import sys
sys.path.insert(1,'/home/pi/Payload-Roket-Watamakara/Kodingan roket/GY-85_Raspberry-Pi-master/i2clibraries')
from i2c_adxl345 import *
from time import *

adxl345 = i2c_adxl345(1)

while True:
    print (adxl345)
    sleep (1)