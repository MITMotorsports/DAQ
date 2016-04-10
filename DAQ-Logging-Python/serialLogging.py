import serial
import time
import os

def cls():
    os.system('cls' if os.name=='nt' else 'clear')
    
ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/ttyUSB1'
ser.open()
print("connected to: " + ser.portstr)

sensors = {}

timestr = time.strftime("%Y%m%d-%H%M%S")
suffix = input("Enter the filename for the log file: ")

if not(suffix == ''):
    file = open('logs/'+timestr+'-'+suffix+'.log', 'w+')
else:
    file = open('logs/'+timestr+'.log', 'w+')

ser.flushInput()
cls()
while True:
    for line in ser:
        try:
            line = line.decode("utf-8")
            data = line.split(',')
            cls()
            print('Sensor\tTimestamp\tData')
            sensors[data[1]] = (data[0], data[2])
            for key in sensors:
                print(key +'\t' + sensors[key][0] + '\t\t' + sensors[key][1])
                file.write(line)
        except:
            pass
ser.close()
