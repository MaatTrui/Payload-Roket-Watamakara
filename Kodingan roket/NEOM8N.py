# Library GPS MAAT MICROPYTHON
# UNTUK GY-GPS (NEO M6N, NEO M7N)
# Written by : Jonathan Richard, Ervin Halimsurya
import math
import machine
class GPS:
    def __init__(self,rxPin,txPin,baudRate):
        self.uart = machine.UART(baudrate = baudRate, bits = 8, parity = None, stop = 1, tx = machine.Pin(txPin), rx = machine.Pin(rxPin))

    def available(self):
        if(self.uart.any()>0):
            return True
        else:
            return False
    def readData(self):
        while(self.uart.any()>0):
            sentence = self.uart.readline()
            if(sentence.startswith(b"$GNGGA", 0, 7)):
                sentence = sentence.decode('utf-8')
                lat,lat_dir,lon,lon_dir,alt= GPS.GPGGA(sentence)
                res_x=str(round(lat,2))+" "+lat_dir
                res_y=str(round(lon,2))+" "+lon_dir
                z=str(round(alt,2))
                return res_x, res_y, z 
            

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
            sd = GPS.dm_to_sd(lat)
            if lat_dir == 'N':
                return +sd
            elif lat_dir == 'S':
                return -sd
            else:
                return 0.

    def longitude(lon,lon_dir):
        '''Longitude in signed degrees (python float)'''
        sd = GPS.dm_to_sd(lon)
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
        
        
    def GPGGA(line):

        line = line.split(",")
        lat_raw=line[2]
        lat_dir=line[3]
        lon_raw=line[4]
        lon_dir=line[5]

        res_lat=GPS.latitude(lat_raw,lat_dir)
        res_lon=GPS.longitude(lon_raw,lon_dir)
        res_alt=GPS.altitude(line[9])

        return res_lat,lat_dir,res_lon,lon_dir,res_alt