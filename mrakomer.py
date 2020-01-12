#!/usr/bin/python3
#
# Logger for mrakomer
# 

import time
import serial
import logging
from logging.handlers import TimedRotatingFileHandler

#port = '/dev/ttyACM0'
port = '/dev/ttyUSB1'

#baud = 115200
baud = 2400
ser = serial.Serial(port, baud, timeout=1)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

logname = "./mrakomer.csv"
handler = TimedRotatingFileHandler(logname, when='h', interval=1, utc=True)
#handler.setLevel(logging.INFO)
#handler.suffix = "%Y%m%d%H%M"
logger.addHandler(handler)

while True:
	reading = ser.readline().decode('utf-8')
	if (len(reading) > 0):
		data = str(round(time.time(),2)) + ',' + reading[:-1]
		print(data)
		logger.info(data)


