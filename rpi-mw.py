#####################################################################
# Aldo Vargas
# 
#
# Purpose:
#	Connects a Raspberry Pi to the MultiWii Flight Controller. Based on 
#	the work made by Drew Brandsen.
#	Using MutliWii Serial Protocol (MSP), requests flight telemetry
#	data. Attitude and RC input values coming from the RX/TX with the
#	purpose to do systems identification, or just act as a data logger.
#	
#
########################################################################


import serial		# for serial communication
import sys			# for user input
import socket		# for UDP communication
import time			# for wait commands
import datetime		# for current time
import struct		# for decoding data strings
import timeit		# for current time
import asyncore 	# for asynchornous udp comm
import SocketServer # for socketserver udp comm
import threading 	# for using threads


##########################################################################
########################### Configuration ################################
##########################################################################
# The following lines define the serial port
##########################################################################

class drone(object):
	FILE 	=	0 	# Save to a timestamped file, the data selected below
	TIME 	= 	1 	# Save the difference of time between all the main functions for perfomance logging
	FLYT 	= 	1 	# Save the flight time in seconds
	ATT 	= 	0 	# Ask and save the attitude of the multicopter
	ALT 	= 	0 	# Ask and save the altitude of the multicopter
	RC  	= 	1 	# Ask and save the pilot commands of the multicopter
	MOT 	= 	0 	# Ask and save the PWM of the motors that the MW is writing to the multicopter
	RAW 	= 	1 	# Ask and save the raw imu data of the multicopter
	CMD 	= 	0 	# Send commands to the MW to control it
	UDP 	=	0 	# Save or use UDP data (to be adjusted)
	ASY 	=	0 	# Use async communicacion
	SCK 	=	0 	# Use regular socket communication
	SCKSRV 	=	0 	# Use socketserver communication
	PRINT 	= 	1 	# Print data to terminal, useful for debugging



##########################################################################
########################### Communication ################################
##########################################################################
# The following lines define the serial port and UDP comms
##########################################################################

ser=serial.Serial()
#ser.port="/dev/tty.usbserial-AM016WP4"	# This is the port that the MultiWii is attached to (for mac & MW home)
ser.port="/dev/tty.usbserial-A101CCVF"	# This is the port that the MultiWii is attached to (for mac & MW office)
#ser.port="/dev/ttyUSB0"	# This is the port that the MultiWii is attached to (for raspberry pie)
ser.baudrate=115200
ser.bytesize=serial.EIGHTBITS
ser.parity=serial.PARITY_NONE
ser.stopbits=serial.STOPBITS_ONE
ser.timeout=0
ser.xonxoff=False
ser.rtscts=False
ser.dsrdtr=False
ser.writeTimeout=2
timeMSP=0.02
#Raspberry pie IP address
udp_ip = "172.30.150.170"
#Mac IP address
#udp_ip = "130.209.27.59"
#udp_ip = "localhost"
udp_port = 51001
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)


###############################
# Initialize Global Variables
###############################
latitude = 0.0
longitude = 0.0
altitude = -0
heading = -0
timestamp = -0
gpsString = -0
numSats = -0
accuracy = -1
beginFlag = 0
roll = 0
pitch = 0
yaw = 0
throttle = 0
angx = 0.0
angy = 0.0
m1 = 0
m2 = 0
m3 = 0
m4 = 0
message = ""
ax = 0
ay = 0
az = 0
gx = 0
gy = 0
gz = 0
magx = 0
magy = 0
magz = 0
elapsed = 0
flytime = 0
udp_mess = ""
udp_mess2 = ""
numOfValues = 0
precision = 2
rcData = [1500, 1500, 1500, 1000] #order -> roll, pitch, yaw, throttle


##########################################################################
################################## UDP ###################################
##########################################################################
# Class for the async UDP server
##########################################################################
class AsyncoreServerUDP(asyncore.dispatcher):
	def __init__(self):
 		asyncore.dispatcher.__init__(self)
		self.create_socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.bind((udp_ip, udp_port))
	
	# Even though UDP is connectionless this is called when it binds to a port
	def handle_connect(self):
		print "Server Started..."
	
	# This is called everytime there is something to read
	def handle_read(self):
		global udp_mess 
		global udp_mess2
		udp_mess=""
		data, addr = self.recvfrom(1024)
		numOfValues = len(data) / 8
		mess=struct.unpack('>' + 'd' * numOfValues, data)
		for x in range(0, numOfValues):
 			udp_mess = udp_mess+" "+str(mess[x])
 		udp_mess2=udp_mess

   # This is called all the time and causes errors if you leave it out.
	def handle_write(self):
 		pass


##########################################################################
################################## UDP ###################################
##########################################################################
# Class for the socketserver 
##########################################################################
class SocketServerHandler(SocketServer.BaseRequestHandler):

	def handle(self):
		global udp_mess 
		global udp_mess2
		global rcData
		udp_mess=""
		numOfValues=0
		timestamp = time.time()
		data = self.request[0].strip()
		socket = self.request[1]
		try:
			numOfValues = len(data) / 8
			mess=struct.unpack('>' + 'd' * numOfValues, data)
			for x in range(0, numOfValues):
 				udp_mess = udp_mess+" "+str(mess[x])
 			for x in range(0,3):
 				if mess[x] is not None:
 					rcData[x]=mess[x]
 			udp_mess2=udp_mess
		except Exception,error:
			#print "SocketServer: "+str(error)
			pass


#####################################################################
###################### MultiWii Serial Protocol######################
#####################################################################
#  The following define the hex values required
#  to make various MSP requests
################################################

BASIC="\x24\x4d\x3c\x00"		#MSG Send Header (to MultiWii)
MSP_IDT=BASIC+"\x64\x64"		#MSG ID: 100
MSP_STATUS=BASIC+"\x65\x65"		#MSG ID: 101
MSP_RAW_IMU=BASIC+"\x66\x66"	#MSG ID: 102
MSP_SERVO=BASIC+"\x67\x67"		#MSG ID: 103
MSP_MOTOR=BASIC+"\x68\x68"		#MSG ID: 104
MSP_RC=BASIC+"\x69\x69"			#MSG ID: 105
MSP_RAW_GPS=BASIC+"\x6A\x6A"	#MSG ID: 106
MSP_ATTITUDE=BASIC+"\x6C\x6C"	#MSG ID: 108
MSP_ALTITUDE=BASIC+"\x6D\x6D"	#MSG ID: 109
MSP_BAT = BASIC+"\x6E\x6E"		#MSG ID: 110
MSP_COMP_GPS=BASIC+"\x71\x71"	#MSG ID: 111
MSP_SET_RC=BASIC+"\xC8\xC8"  	#MSG ID: 200

CMD2CODE = {
'MSP_IDENT':100,
'MSP_STATUS':101,
'MSP_RAW_IMU':102,
'MSP_SERVO':103,
'MSP_MOTOR':104,
'MSP_RC':105,
'MSP_RAW_GPS':106,
'MSP_COMP_GPS':107,
'MSP_ATTITUDE':108,
'MSP_ALTITUDE':109,
'MSP_ANALOG':110,
'MSP_RC_TUNING':111,
'MSP_PID':112,
'MSP_BOX':113,
'MSP_MISC':114,
'MSP_MOTOR_PINS':115,
'MSP_BOXNAMES':116,
'MSP_PIDNAMES':117,
'MSP_WP':118,
'MSP_BOXIDS':119,

'MSP_SET_RAW_RC':200,
'MSP_SET_RAW_GPS':201,
'MSP_SET_PID':202,
'MSP_SET_BOX':203,
'MSP_SET_RC_TUNING':204,
'MSP_ACC_CALIBRATION':205,
'MSP_MAG_CALIBRATION':206,
'MSP_SET_MISC':207,
'MSP_RESET_CONF':208,
'MSP_SET_WP':209,
'MSP_SWITCH_RC_SERIAL':210,
'MSP_IS_SERIAL':211,
'MSP_DEBUG':254,
}

################################################
# ALTITUDE(msp)
#	receives: msp altitude message
#	outputs:  prints data in nice format
#	returns:  Angle in X, Y and heading
################################################
def ATTITUDE(msp):
	global angx
	global angy
	global heading
	if str(msp) == "":
		#print(msp_hex)
		#print("Header: " + msp_hex[:6])
		#payload = int(msp_hex[6:8])
		#print("Payload: " + msp_hex[6:8])
		#print("Code: " + msp_hex[8:10])
		#print("RC data unavailable")
		return
	else:
		msp_hex = msp.encode("hex")
		if msp_hex[10:14] == "":
			#print("angx unavailable")
			return
		else:
			angx = littleEndian(msp_hex[10:14])/10.0

		if msp_hex[14:18] == "":
			#print("angy unavailable")
			return
		else:
			angy = littleEndian(msp_hex[14:18])/10.0

		if msp_hex[18:22] == "":
			#print("head unavailable")
			return
		else:
			heading = littleEndian(msp_hex[18:22])+5


################################################
# ALTITUDE(msp)
#	receives: msp altitude message
#	outputs:  prints data in nice format
#	returns:  Altitude in CSV
################################################
def ALTITUDE(msp):
	global beginFlag
	global altitude
	if str(msp) == "":
		#print(msp_hex)
		#print("Header: " + msp_hex[:6])
		#payload = int(msp_hex[6:8])
		#print("Payload: " + msp_hex[6:8])
		#print("Code: " + msp_hex[8:10])
		#print("Altitude unavailable")
		beginFlag = 1	# set flag to 1
		return
	else:
		msp_hex = msp.encode("hex")
		if msp_hex[10:18] == "":
			#print("Altitude unavailable2")
			beginFlag = 1
			return
		else:
			altitude = float(littleEndian(msp_hex[10:18]))/100
			#print("Altitude: " + str(altitude) + " meters")
			return


################################################
# RC(msp)
#\t   receives: msp RC message
#\t   outputs:  prints data in nice format
#\t   returns:  Roll/Pitch/Yaw/Throttle
################################################
def RC(msp):
	global roll
	global pitch
	global yaw
	global throttle
	if str(msp) == "":
		#print(msp_hex)
		#print("Header: " + msp_hex[:6])
		#payload = int(msp_hex[6:8])
		#print("Payload: " + msp_hex[6:8])
		#print("Code: " + msp_hex[8:10])
		#print("RC data unavailable")
		return
	else:
		msp_hex = msp.encode("hex")

		if msp_hex[10:14] == "":
			#print("roll unavailable")
			return
		else:
			roll = float(littleEndian(msp_hex[10:14]))

		if msp_hex[14:18] == "":
			#print("pitch unavailable")
			return
		else:
			pitch = float(littleEndian(msp_hex[14:18]))

		if msp_hex[18:22] == "":
			#print("yaw unavailable")
			return
		else:
			yaw = float(littleEndian(msp_hex[18:22]))

		if msp_hex[22:26] == "":
			#print("throttle unavailable")
			return
		else:
			throttle = float(littleEndian(msp_hex[22:26]))
			return
		#print(str(roll) + " " + str(pitch) + " " + str(yaw) + " " + str(throttle))


################################################
# MOTORS(msp)
#\t   receives: msp MOTORS message
#\t   outputs:  prints data in nice format
#\t   returns:  motor 1/motor 2/motor 3/motor 4
################################################
def MOTORS(msp):
	global m1
	global m2
	global m3
	global m4
	if str(msp) == "":
		#print(msp_hex)
		#print("Header: " + msp_hex[:6])
		#payload = int(msp_hex[6:8])
		#print("Payload: " + msp_hex[6:8])
		#print("Code: " + msp_hex[8:10])
		#print("RC data unavailable")
		return
	else:
		msp_hex = msp.encode("hex")

		if msp_hex[10:14] == "":
			#print("motor 1 unavailable")
			return
		else:
			m1 = float(littleEndian(msp_hex[10:14]))

		if msp_hex[14:18] == "":
			#print("motor 2 unavailable")
			return
		else:
			m2 = float(littleEndian(msp_hex[14:18]))

		if msp_hex[18:22] == "":
			#print("motor 3 unavailable")
			return
		else:
			m3 = float(littleEndian(msp_hex[18:22]))

		if msp_hex[22:26] == "":
			#print("motor 4 unavailable")
			return
		else:
			m4 = float(littleEndian(msp_hex[22:26]))
			return
		#print(str(roll) + " " + str(pitch) + " " + str(yaw) + " " + str(throttle))


################################################
# RAW(msp)
#\t   receives: msp raw message
#\t   outputs:  prints data in nice format
#\t   returns:  ax/ay/az/gx/gy/gz/magx/magy/magz
################################################
def RAW(msp):
	global ax
	global ay
	global az
	global gx
	global gy
	global gz
	global magx
	global magy
	global magz
	if str(msp) == "":
		#print(msp_hex)
		#print("Header: " + msp_hex[:6])
		#payload = int(msp_hex[6:8])
		#print("Payload: " + msp_hex[6:8])
		#print("Code: " + msp_hex[8:10])
		#print("RC data unavailable")
		return
	else:
		msp_hex = msp.encode("hex")
		if msp_hex[10:14] == "":
			#print("ax unavailable")
			return
		else:
			ax = float(littleEndian(msp_hex[10:14]))
		
		if msp_hex[14:18] == "":
			#print("ay unavailable")
			return
		else:
			ay = float(littleEndian(msp_hex[14:18]))

		if msp_hex[18:22] == "":
			#print("az unavailable")
			return
		else:
			az = float(littleEndian(msp_hex[18:22]))

		if msp_hex[22:26] == "":
			#print("gx unavailable")
			return
		else:
			gx = float(littleEndian(msp_hex[22:26]))
		
		if msp_hex[26:30] == "":
			#print("gy unavailable")
			return
		else:
			gy = float(littleEndian(msp_hex[26:30]))

		if msp_hex[30:34] == "":
			#print("gz unavailable")
			return
		else:
			gz = float(littleEndian(msp_hex[30:34]))

		if msp_hex[34:38] == "":
			#print("magx unavailable")
			return
		else:
			magx = float(littleEndian(msp_hex[34:38]))
		
		if msp_hex[38:42] == "":
			#print("magy unavailable")
			return
		else:
			magy = float(littleEndian(msp_hex[38:42]))

		if msp_hex[42:46] == "":
			#print("magz unavailable")
			return
		else:
			magz = float(littleEndian(msp_hex[42:46]))
			return
		#print(str(roll) + " " + str(pitch) + " " + str(yaw) + " " + str(throttle))  


#############################################################
# sendData(data_length, code, data)
#	receives: the data length of the message, the code to send and the actual data to send
#	outputs:  errors
#	
#############################################################


def sendData(data_length, code, data):
	checksum = 0
	total_data = ['$', 'M', '<', data_length, code] + data
	for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
		checksum = checksum ^ ord(i)

	total_data.append(checksum)

	#print ser
	try:
		b = None
		b = ser.write(struct.pack('<3c2B%dhB' % len(data), *total_data))
		#ser.flushInput()	# cleans out the serial port
		#ser.flushOutput()
	except Exception, ex:
		print 'send data error'
	return b


#############################################################
# littleEndian(value)
#	receives: a parsed, hex data piece
#	outputs:  the decimal value of that data
#	function: swaps byte by byte to convert little
#			endian to big endian
#	function: calls 2's compliment to convert to decimal
#	returns:  The integer value
#############################################################
def littleEndian(value):
	length = len(value)	# gets the length of the data piece
	actual = ""
	for x in range(0, length/2):	#go till you've reach the halway point
		actual += value[length-2-(2*x):length-(2*x)]	#flips all of the bytes (the last shall be first)
		x += 1
	intVal = twosComp(actual)	# sends the data to be converted from 2's compliment to int
	return intVal				# returns the integer value


###################################################################
# twosComp(hexValue)
#	receives: the big endian hex value (correct format)
#	outputs:  the decimal value of that data
#	function: if the value is negative, swaps all bits
#			up to but not including the rightmost 1.
#			Else, just converts straight to decimal.
#			(Flip all the bits left of the rightmost 1)
#	returns:  the integer value
###################################################################
def twosComp(hexValue):
	firstVal = int(hexValue[:1], 16)
	if firstVal >= 8:	# if first bit is 1
		bValue = bin(int(hexValue, 16))
		bValue = bValue[2:]	# removes 0b header
		newBinary = []
		length = len(bValue)
		index = bValue.rfind('1')	# find the rightmost 1
		for x in range(0, index+1):	# swap bits up to rightmost 1
			if x == index:		#if at rightmost one, just append remaining bits
				newBinary.append(bValue[index:])
			elif bValue[x:x+1] == '1':
				newBinary.append('0')
			elif bValue[x:x+1] == '0':
				newBinary.append('1')
			x += 1
		newBinary = ''.join(newBinary) 	# converts char array to string
		finalVal = -int(newBinary, 2)	# converts to decimal
		return finalVal
			
	else:		# if not a negative number, simply convert to decimal
		return int(hexValue, 16)


#############################################################
# askATT()
#	receives: nothing
#	outputs:  nothing
#	function: Do everything to ask the MW for data and save it on globals 
#	returns:  nothing
#############################################################
def askATT():
	ser.flushInput()	# cleans out the serial port
	ser.flushOutput()
	ser.write(MSP_ATTITUDE)	# sends MSP request
	time.sleep(timeMSP)	# gives adaquate time between MSP TX & RX
	response=ser.readline()	# reads MSP response
	ATTITUDE(response)	# sends to ATTITUDE to parse and update global variables


#############################################################
# askRC()
#	receives: nothing
#	outputs:  nothing
#	function: Do everything to ask the MW for data and save it on globals 
#	returns:  nothing
#############################################################
def askRC():
	ser.flushInput()	# cleans out the serial port
	ser.flushOutput()
	ser.write(MSP_RC)	# gets RC information
	time.sleep(timeMSP)
	response = ser.readline()
	RC(response)


#############################################################
# askALT()
#	receives: nothing
#	outputs:  nothing
#	function: Do everything to ask the MW for data and save it on globals 
#	returns:  nothing
#############################################################
def askALT():
	ser.flushInput()	# cleans out the serial port
	ser.flushOutput()
	ser.write(MSP_ALTITUDE)	# gets ALTITUDE data
	time.sleep(timeMSP)
	response=ser.readline()
	ALTITUDE(response)


#############################################################
# askMOTOR()
#   receives: nothing
#   outputs:  nothing
#   function: Do everything to ask the MW for data and save it on globals 
#   returns:  nothing
#############################################################
def askMOTOR():
	ser.flushInput()	# cleans out the serial port
	ser.flushOutput()
	ser.write(MSP_MOTOR) # gets motors data
	time.sleep(timeMSP)
	response=ser.readline()
	MOTORS(response)


#############################################################
# askRAW()
#   receives: nothing
#   outputs:  nothing
#   function: Do everything to ask the MW for data and save it on globals 
#   returns:  nothing
#############################################################
def askRAW():
	ser.flushInput()	# cleans out the serial port
	ser.flushOutput()
	ser.write(MSP_RAW_IMU) # gets raw data
	time.sleep(timeMSP)
	response=ser.readline()
	RAW(response)


#############################################################
# getUDP()
#   receives: nothing
#   outputs:  nothing
#   function: Receive, decode and print a udp packet data 
#   returns:  nothing
#############################################################
def getUDP():
	global udp_mess 
	global udp_mess2
	udp_mess=""
	data, addr = sock.recvfrom(1024)
	numOfValues = len(data) / 8
	mess=struct.unpack('>' + 'd' * numOfValues, data)
	for x in range(0, numOfValues):
 		udp_mess = udp_mess+" "+str(mess[x])
 	udp_mess2=udp_mess


#############################################################
# SetRC()
#   receives: nothing
#   outputs:  nothing
#   function: Sends RC raw data to Multiwii 
#   returns:  nothing
#############################################################
def setRC():
	rcData = [ int(x) for x in rcData ]
	print rcData


####################################################################
####################### MAIN #######################################
####################################################################
# main()
#	receives: -
#	outputs:  -
#	function: opens serial port
#		 loops msp commands
####################################################################
def main():
	global beginFlag
	global udp_mess2 #Need to be able to modify the value of this variable

	if drone.ASY:
		print ("Beginning Asyncore UDP server on ")+str(udp_ip)
		AsyncoreServerUDP()
		loop_thread = threading.Thread(target=asyncore.loop, name="Asyncore Loop")
		loop_thread.start()
	if drone.SCK:
		print ("Beginning regular UDP server on ")+str(udp_ip)
		sock.bind((udp_ip, udp_port))
	if drone.SCKSRV:
		print ("Beginning UDP socketserver on ")+str(udp_ip)
		server = SocketServer.UDPServer((udp_ip, udp_port), SocketServerHandler)
		loop_thread = threading.Thread(target=server.serve_forever, name="SocketServer Loop")
		loop_thread.start()

	print ("Beginning Multiwii - wait 14 seconds...")

	try:
		ser.open()		# Opens the MultiWii serial port

	except Exception,e:	# catches any errors with opening serial ports
		print("Error open serial port: "+str(e))
		exit()
	
	if ser.isOpen():
		time.sleep(14)		# Gives time for the MultiWii to calibrate and begin sending live info

		if drone.FILE:
			st = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d+%H-%M-%S')+".csv"
			file = open("data/"+st, "w")
		if drone.FLYT:
			flytime = timeit.default_timer()
		
		try:
			while True:
		#########################################################################
		# Loop
		#########################################################################
				#Start timing
				timestamp = timeit.default_timer()

				if drone.CMD and drone.UDP:
					setRC()
				if drone.ATT:
					askATT()
				if drone.ALT:
					askALT()
				if drone.RC:
					askRC()
				if drone.MOT:
					askMOTOR()
				if drone.RAW:
					askRAW()
				if drone.SCK:
					getUDP()

				#Finish timing and save the time difference
				diff = timeit.default_timer() - timestamp
				elapsed = timeit.default_timer() - flytime

				if beginFlag != 1:	# Won't send any data until both altitude and heading are valid data
					#Start adding all data to a variable
					if drone.TIME:
						message = str(round(diff,precision))
					#Save elapsed time
					if drone.FLYT:
						message = message+" "+str(elapsed)
					#save attitude
					if drone.ATT:
						message = message+" "+str(angx)+" "+str(angy)+" "+str(heading)
					#save pilot commands
					if drone.RC:
						message = message+" "+str(roll)+" "+str(pitch)+" "+str(yaw)+" "+str(throttle)
					#save altitude
					if drone.ALT:
						message = message+" "+str(altitude)
					#save motors
					if drone.MOT:
						message = message+" "+str(m1)+" "+str(m2)+" "+str(m3)+" "+str(m4)
					#save raw
					if drone.RAW:
						message = message+" "+str(ax)+" "+str(ay)+" "+str(az)+" "+str(gx)+" "+str(gy)+" "+str(gz)+" "+str(magx)+" "+str(magy)+" "+str(magz)
					#save udp
					if drone.UDP:
						if udp_mess == "":
							message = message+" "+udp_mess2
							udp_mess2=""
						message = message+" "+udp_mess
					#print to terminal
					if drone.PRINT:
						print(message)	
					# print in CSV file
					if drone.FILE:
						file.write(message+"\n")

				else:			# If invalid, continue looping
					beginFlag = 0	# resets the flag
			ser.close()
			file.close()
		
		except Exception,e1:	# Catches any errors in the serial communication
			print("Error on main: "+str(e1))
	else:
		print("Cannot open serial port")

#-----------------------------------------------------------------------
if __name__=="__main__":
	main()