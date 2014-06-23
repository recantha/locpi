#!/usr/bin/python

import RPi.GPIO as GPIO
#from PyComms import mpu6050
#import lcddriver
import math
import commands
import os
import time
import threading
from smbus import SMBus
from datetime import datetime
from gps import *
import sys
import termios
import tty

version_no = "0.1"
version_date = "June 2014"

LCD_ENABLE = False
GPS_ENABLE = True
DEBUG=1

###############################################################
# LCD
def display(line_1, line_2="                    ", line_3="                    ", line_4="                    "):

	line_1 = line_1.ljust(20, " ")
	line_2 = line_2.ljust(20, " ")
	line_3 = line_3.ljust(20, " ")
	line_4 = line_4.ljust(20, " ")

	if LCD_ENABLE:
		lcd.lcd_display_string(line_1, 1)
		lcd.lcd_display_string(line_2, 2)
		lcd.lcd_display_string(line_3, 3)
		lcd.lcd_display_string(line_4, 4)

	if DEBUG == 1:
		print "--------------------"
		print line_1
		print line_2
		print line_3
		print line_4
		print "--------------------"


###############################################################
# SESSION
def readHostname():
        local_hostname=socket.gethostname()
        return local_hostname

def readIPaddresses():
        ips = commands.getoutput("/sbin/ifconfig | grep -i \"inet\" | grep -iv \"inet6\" | " + "awk {'print $2'} | sed -ne 's/addr\:/ /p'")
        addrs = ips.split('\n')

        return addrs

###############################################################
# HMC reading
def readHMC5883L():
        try:
                data = hmc.getHeading()
                reading = {}
                reading['yaw'] = "%.0f" % data['z']
                reading['pitch'] = "%.0f" % data['y']
                reading['roll'] = "%.0f" % data['x']

        except:
                reading = {}
                reading['yaw'] = -1
                reading['pitch'] = -1
                reading['roll'] = -1

        return reading

###############################################################
# MPU reading
def readMPU6050():
        # Sensor initialization

        # get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize()

        try:
                while True:
                        # Get INT_STATUS byte
                        mpuIntStatus = mpu.getIntStatus()

                        if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently)
                                # get current FIFO count
                                fifoCount = mpu.getFIFOCount()

                                # check for overflow (this should never happen unless our code is too inefficient)
                                if fifoCount == 1024:
                                        # reset so we can continue cleanly
                                        mpu.resetFIFO()
                                        #print('FIFO overflow!')


                                # wait for correct available data length, should be a VERY short wait
                                fifoCount = mpu.getFIFOCount()
                                while fifoCount < packetSize:
                                        fifoCount = mpu.getFIFOCount()

                                result = mpu.getFIFOBytes(packetSize)
                                q = mpu.dmpGetQuaternion(result)
                                g = mpu.dmpGetGravity(q)
                                ypr = mpu.dmpGetYawPitchRoll(q, g)

                                reading = {}
                                reading['yaw'] = "%.2f" % (ypr['yaw'] * 180 / math.pi)
                                reading['pitch'] = "%.2f" % (ypr['pitch'] * 180 / math.pi)
                                reading['roll'] = "%.2f" % (ypr['roll'] * 180 / math.pi)

                                # track FIFO count here in case there is > 1 packet available
                                # (this lets us immediately read more without waiting for an interrupt)
                                fifoCount -= packetSize
                                # break when you have a reading
                                return reading
        except:
                reading = {}
                reading['yaw'] = -1
                reading['pitch'] = -1
                reading['roll'] = -1

                return reading


###############################################################
# GPS
class GpsPoller(threading.Thread):
        def __init__(self):
                threading.Thread.__init__(self)
                global gpsd #bring it in scope
                gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
                self.current_value = None
                self.running = True #setting the thread running to true

        def run(self):
                global gpsd
                while gpsp.running:
                        gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer

###########################################################
def readCoordinates():
        try:
                lat = gpsd.fix.latitude
                lon = gpsd.fix.longitude
                speed = gpsd.fix.speed
                utc = gpsd.utc
                alt = gpsd.fix.altitude

                if (math.isnan(lat)):
                        lat = "No fix"

                if (math.isnan(lon)):
                        lon = "No fix"

                if (math.isnan(speed)):
                        speed = "No fix"
                else:
                        speed = "%s m/s" % speed

                if (utc):
                        pass
                else:
                        utc = "No fix"

                if (math.isnan(alt)):
                        alt = "No fix"
                else:
                        alt = "%s metres" % alt

                sats = gpsd.satellites

                coords = [lat, lon, utc, alt, speed, sats]

        except (KeyboardInterrupt, SystemExit):
                pass

        return coords


###############################################################
# Read system temperatures
def readSystemTemperatures():
	try:
		reading = commands.getoutput("./getSystemTemperature.sh")
		readings = reading.splitlines();

	except:
		readings = [-1, -1]

	return readings

###############################################################
# INIT SECTION
###############################################################
# GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LCD
if LCD_ENABLE:
	try:
		lcd = lcddriver.lcd()
	except:
		print "LCD failed to initialise"

# ACCEL NUMBER 2
#try:
#	mpu = mpu6050.MPU6050()
#	mpu.dmpInitialize()
#	mpu.setDMPEnabled(True)
#
#except:
#	print "MPU failed to initialise"

# I2C bus
bus = SMBus(1)

#############################################################
# Start-up routine
display("####################", "locpi v" + version_no, "Starting up...", "####################")
time.sleep(0.5)

# GPS
if GPS_ENABLE:
	display("####################", "Initialising GPS", "", "####################")
	os.system("./enable_gps.sh")
	time.sleep(4)
	display("####################", "GPS should be up", "", "####################")
	gpsd = None
	gpsp = GpsPoller()
	gpsp.start()

###############################################################
# MAIN
if __name__ == "__main__":
	while True:
		try:
			coords = readCoordinates()
			display("Latitude %s" % coords[0], "Longitude %s" % coords[1], "Altitude %s" % coords[3], "Speed %s" % coords[4])
			time.sleep(0.5)

			#ypr = readMPU6050()
			#display("MPU accelerometer", "Yaw: " + ypr['yaw'], "Pitch: " + ypr['pitch'], "Roll: " + ypr['roll'])
			#time.sleep(0.05)

		except KeyboardInterrupt:
			gpsp.running = False
			gpsp.join()
			GPIO.cleanup()
			raise

		except:
			print "Error"
			exit(0)
			raise
