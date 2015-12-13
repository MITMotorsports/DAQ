import serial
import time
import os
from elasticsearch import Elasticsearch

def cls():
    os.system('cls' if os.name=='nt' else 'clear')

def calculated_time(milliseconds):
	pass


ser = serial.Serial()
ser.baudrate = 9600
ser.port = '/dev/tty.usbserial-AI03L0IZ'
ser.open()
print("connected to: " + ser.portstr)

#ser = ['2015-12-05T13:11:59Z,TMP,123 123 123', '2015-12-05T13:09:59Z,TMP,122 123 123']
es = Elasticsearch()
sensors = {}

timestr = time.strftime("%Y%m%d-%H%M%S")
suffix = raw_input("Enter the filename for the log file: ")

if not(suffix == ''):
	file = open('logs/'+timestr+'-'+suffix+'.log', 'w+')
else:
	file = open('logs/'+timestr+'.log', 'w+')

while True:
    for line in ser:
	data = line.split(',')
	if data[1] == 'RTC':
		calculate_time(data[0])
	else:
		res = es.index(index="sensor", doc_type = 'data', body= { 'sensor' : data[1], '_timestamp': data[0], 'data': data[2]})
		cls()
 		print 'Sensor\tTimestamp\t\tData'		
		sensors[data[1]] = (data[0], data[2])
		for key in sensors:
			print key +'\t' + sensors[key][0] + '\t' + sensors[key][1]
		file.write(line)
ser.close()


