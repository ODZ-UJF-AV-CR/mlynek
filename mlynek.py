#!/usr/bin/python3
#
# Logger for mlynek
# 

import time
import serial
import logging
from logging.handlers import TimedRotatingFileHandler

#port = '/dev/ttyACM0'
port = '/dev/ttyUSB0'

#baud = 115200
baud = 9600
ser = serial.Serial(port, baud, timeout=1)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

logname = "./mlynek.csv"
handler = TimedRotatingFileHandler(logname, when='m', interval=1, utc=True)
#handler.setLevel(logging.INFO)
#handler.suffix = "%Y%m%d%H%M"
logger.addHandler(handler)

while True:
	reading = ser.readline().decode('utf-8')
	if (len(reading) > 0):
		data = str(round(time.time(),2)) + ',' + reading[:-1]
		print(data)
		logger.info(data)


