
#####################################################################
# Aldo Vargas
# 
#
# Purpose:
#	Connects a Raspberry Pi to the MultiWii Flight Controller.
#	Using MutliWii Serial Protocol (MSP), requests flight telemetry
#	data such as Altitude, Attitude (heading), and RC input values
#	It then sends that data over TCP/IP to be received by the base
#	station.
#
########################################################################


import serial	# for serial communication
import sys		# for user input
import socket	# for TCP/IP communication
import time		# for wait commands
import datetime	# for current time

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

##########################################################################
########################### Communication ################################
##########################################################################
# The following lines define the serial ports and TCP/IP with the correct parameters
##########################################################################
#number = raw_input("Enter last digit of IP: ") # Prompts for IP address
#TCP_IP = '10.0.1.'+number     # This is the IP address of the laptop (client) 10.0.1.4 = Drew, 10.0.1.6 = Jared
TCP_IP = '10.0.1.6'
TCP_PORT = 5005			  # Set a port, make this consistent on both ends

ser=serial.Serial()
ser.port="/dev/tty.usbserial-AM016WP4"	# This is the port that the MultiWii is attached to
ser.baudrate=115200
ser.bytesize=serial.EIGHTBITS
ser.parity=serial.PARITY_NONE
ser.stopbits=serial.STOPBITS_ONE
ser.timeout=0
ser.xonxoff=False
ser.rtscts=False
ser.dsrdtr=False
ser.writeTimeout=2

#ser2 = serial.Serial()		# Readies the serial port
#ser2.port="/dev/ttyAMA0"	# Defines the serial port to open (GPIO pins on Raspberry Pi)

# ONLY NEED THIS CODE IF GPS REVERTS TO DEFAULT #
#ser2.baudrate=57600		# Baud rate of GPS (Adafruit/sainsmart=9600, Mediatek = 38400, GSTAR = 57600)
#ser2.open()
#ser2.flushOutput()
#ser2.write("$PMTK251,115200*1F\r\n")	# Changes BAUD rate to 115200
#time.sleep(1)
#ser2.close()
##########################################################
#ser2.baudrate = 115200				# Reopens port at new baud rate (115200)
#ser2.open()
#ser2.flush()
#ser2.write("$PMTK220,1000*1F\r\n") # Changes refresh rate to 1Hz
#ser2.write("$PMTK220,100*2F\r\n")	# Changes refresh rate to 10Hz


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

################################################
# ATTITUDE(msp)
#	receives: msp attitude message
#	outputs:  prints heading in nice format
#	returns:  Heading in CSV
################################################
def ATTITUDE(msp):
	global beginFlag
	global heading
	if str(msp) == "":	
		#print("Heading: unavailable")		# if the heading data is not there
		beginFlag = 1				# set begin flag to 1
		return
	else:
		msp_hex = msp.encode("hex")		# Converts the data into Hex
		#print(msp_hex)
		#print("Header: " + msp_hex[:6])
		#payload = int(msp_hex[6:8])
		#print("Payload: " + msp_hex[6:8])
		#print("Code: " + msp_hex[8:10])
		#angx1 = littleEndian(msp_hex[10:14])	# Sends the angle X to be converted into int
		#angx = angx1/10.0			# Converts into degrees
		#print("AngleX: " + str(angx) + " degrees")	# Prints data
		#angy1 = littleEndian(msp_hex[14:18])	# Sends the angle Y to be converted into int
		#angy = angy1/10.0			# Converts into degrees
		#print("AngleY: " + str(angy) + " degrees")	# Prints data	
		if msp_hex[18:22] == "":
			#print("Heading: unavailable2")
			beginFlag = 1
			return
		else:
			angx = littleEndian(msp_hex[10:14])/10.0
			angy = littleEndian(msp_hex[14:18])/10.0
			heading = littleEndian(msp_hex[18:22])+5	# else, send heading to be converted into int
			return
		#print("Heading: " + str(heading) + " degrees")	#Prints data


################################################
# ALTITUDE2(msp)
#	receives: msp altitude message
#	outputs:  prints data in nice format
#	returns:  Altitude in CSV
################################################
def ATTITUDE2(msp):
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
#       receives: msp RC message
#       outputs:  prints data in nice format
#       returns:  Roll/Pitch/Yaw/Throttle
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
                        print("roll unavailable")
                        return
                else:
                        roll = float(littleEndian(msp_hex[10:14]))

                if msp_hex[14:18] == "":
                        print("pitch unavailable")
                        return
                else:
                        pitch = float(littleEndian(msp_hex[14:18]))

                if msp_hex[18:22] == "":
                        print("yaw unavailable")
                        return
                else:
                        yaw = float(littleEndian(msp_hex[18:22]))

                if msp_hex[22:26] == "":
                        print("throttle unavailable")
                        return
                else:
                        throttle = float(littleEndian(msp_hex[22:26]))
                        return
                #print(str(roll) + " " + str(pitch) + " " + str(yaw) + " " + str(throttle))


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

#############################################################################
################################# GPS #######################################
#############################################################################
# GPS()
#	receives: nil
#	outputs:  nil
#	function: reads values in from serial port, gets the required data, and
#			then updates the global variables which are printed in main below
#	returns:  updates global variables; Longitude, Latitude, and Timestamp
#############################################################################
def GPS():
	line = []
	ser2.flushInput()
	ser2.flushOutput()
	global longitude
	global latitude
	global timestamp
	global numSats
	global accuracy
	count = 0
	while True:
		for x in ser2.read():	# as long as their are characters to read...
			line.append(x)	# append the new character to the line array
			#print(x)
			if x == '\n':	# if a character is a newline char,
				string = ''.join(line)	# convert the array into a string
				#print(string)
				if string.count("$GPGGA") != 0:	# if the string starts with GPGGA - sentence with all the needed information
					parsed_string = string.split(',')	# split the string at every comma (CSV type sentence)
							
					time = string.split(',')[1]		# time is the first split
					if time != "":
						timestamp = time
					lat = string.split(',')[2]      # longitude
					if lat != "":
						latitude = lat
					lon = string.split(',')[4]      # latitude
					if lon != "":
						longitude = lon
					num = string.split(',')[7]		# number of satellites
					if num != "":
						numSats = num
					accuracy = string.split(',')[8]	# Current location accuracy (meters)
					#altitude = string.split(',')[9] + " " + string.split(',')[10]	# altitude + units
					count = 0
					return
				else:		# if not a GPGGA sentence, clear out the char array and keep listening.
					line = []
					count = count + 1
					if (count >= 10):
					    return;
			
####################################################################
####################### Communication ##############################
####################################################################
# main()
#	receives: -
#	outputs:  -
#	function: opens serial port, opens TCP/IP port,
#		 loops msp commands, loops GPS function, sends data over TCP/IP
#	function: calls print functions to output correct data
####################################################################
def main():
	global beginFlag
	##########################################################################
	# The following lines define the TCP/IP port with the correct parameters
	##########################################################################
	#s = socket.socket()		# Opens the TCP/IP stream
	print("Connecting to '"+TCP_IP+"'")
	#s.connect((TCP_IP, TCP_PORT))	# Connects to the given IP address and Port
	#print s.recv(1024)		# Receives a message from the other computer (1024 buffer size)	 
	
	print ("Beginning in 8 seconds...")

	try:
		ser.open()		# Opens the MultiWii serial port
		#ser2.open()		# Opens the GPS serial port

	except Exception,e:	# catches any errors with opening serial ports
		print("Error open serial port: "+str(e))
		exit()
	
	if ser.isOpen(): #& ser2.isOpen():
		#time.sleep(13.2) 	# Exact time for Altitude data to become live.
		time.sleep(8)		# Gives time for the MultiWii to calibrate and begin sending live info
		print("Serial port is open at"+ser.portstr)
		#print("Serial port is open at"+ser2.portstr)
		try:
			while True:
		#########################################################################
		# Sends and Recieves MSP commands then sends received data over TCP/IP
		#########################################################################
				ser.flushInput()	# cleans out the serial port
				ser.flushOutput()

				ser.write(MSP_ATTITUDE)	# sends MSP request
				time.sleep(0.02)	# gives adaquate time between MSP TX & RX
				response=ser.readline()	# reads MSP response
				ATTITUDE2(response)	# sends to ATTITUDE to parse and update global variables
				ser.flushInput()	# cleans out the serial port
				ser.flushOutput()
					
				#ser.write(MSP_RC)	# gets RC information
				#time.sleep(0.01)
				#response = ser.readline()
				#RC(response)
				#ser.flushInput();
				#ser.flushOutput();

				#ser.write(MSP_ALTITUDE)	# gets ALTITUDE data
				#time.sleep(0.01)
				#response=ser.readline()
				#ALTITUDE(response)
				#ser.flushInput();
				#ser.flushOutput();

				#GPS();		# Calls GPS function to get GPS data

				if beginFlag != 1:	# Won't send any data until both altitude and heading are valid data
					#print("A"+str(latitude)+","+str(longitude)+","+str(altitude)+","+str(heading)+","+str(timestamp)+"Z"+str(numSats)+","+str(accuracy)+","+str(pitch)+","+str(roll)+","+str(yaw)+","+str(throttle)+"W")	# print in CSV
					print(str(timestamp)+" "+str(angx)+" "+str(angy)+" "+str(heading))	# print in CSV
					#s.send("A"+str(latitude)+","+str(longitude)+","+str(altitude)+","+str(heading)+","+str(timestamp)+"Z"+str(numSats)+","+str(pitch)+","+str(roll)+","+str(yaw)+","+str(throttle)+"W")	# send in CSV
				else:			# If invalid, continue looping
					beginFlag = 0	# resets the flag
		       	ser.close()
				#ser2.close()

	    	except Exception,e1:	# Catches any errors in the serial communication
	        	print("Error communicating..."+str(e1))
	else:
    		print("Cannot open serial port")

#-----------------------------------------------------------------------
if __name__=="__main__":
	main()