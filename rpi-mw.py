
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


import serial	# for serial communication
import sys		# for user input
import socket	# for TCP/IP communication
import time		# for wait commands
import datetime	# for current time
import struct


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
# The following lines define the serial port
##########################################################################

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
    'MSP_IDENT'           :     100,
    'MSP_STATUS'          :     101,
    'MSP_RAW_IMU'         :     102,
    'MSP_SERVO'           :     103,
    'MSP_MOTOR'           :     104,
    'MSP_RC'              :     105,
    'MSP_RAW_GPS'         :     106,
    'MSP_COMP_GPS'        :     107,
    'MSP_ATTITUDE'        :     108,
    'MSP_ALTITUDE'        :     109,
    'MSP_ANALOG'          :     110,
    'MSP_RC_TUNING'       :     111,
    'MSP_PID'             :     112,
    'MSP_BOX'             :     113,
    'MSP_MISC'            :     114,
    'MSP_MOTOR_PINS'      :     115,
    'MSP_BOXNAMES'        :     116,
    'MSP_PIDNAMES'        :     117,
    'MSP_WP'              :     118,
    'MSP_BOXIDS'          :     119,

    'MSP_SET_RAW_RC'      :     200,
    'MSP_SET_RAW_GPS'     :     201,
    'MSP_SET_PID'         :     202,
    'MSP_SET_BOX'         :     203,
    'MSP_SET_RC_TUNING'   :     204,
    'MSP_ACC_CALIBRATION' :     205,
    'MSP_MAG_CALIBRATION' :     206,
    'MSP_SET_MISC'        :     207,
    'MSP_RESET_CONF'      :     208,
    'MSP_SET_WP'          :     209,
   'MSP_SWITCH_RC_SERIAL' :     210,
   'MSP_IS_SERIAL'        :     211,
    'MSP_DEBUG'           :     254,
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
        ser.flushInput()	# cleans out the serial port
        ser.flushOutput()
    except Exception, ex:
        print 'send data is_valid_serial fail'
        multi_info['is_valid_serial'] = False;
        connect()
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

def multiwiiCMD():
	rc_data = [1500, 1500, 2000, 1000 ]
	sendData(16, CMD2CODE['MSP_SET_RAW_RC'], rc_data)
	time.sleep(1)
	rc_data = [1500, 1500, 2000, 1000 ]
	sendData(16, CMD2CODE['MSP_SET_RAW_RC'], rc_data)
	print('Arming... 3')
	time.sleep(1)
	print('Arming... 2')
	time.sleep(1)
	print('Arming... 1')
	time.sleep(1)
	rc_data = [1500, 1500, 1500, 1000 ]
	sendData(16, CMD2CODE['MSP_SET_RAW_RC'], rc_data)
    
			
####################################################################
####################### MAIN #######################################
####################################################################
# main()
#	receives: -
#	outputs:  -
#	function: opens serial port
#		 loops msp commands
#	function: calls print functions to output correct data
####################################################################
def main():
	global beginFlag
	
	print ("Beginning in 8 seconds...")

	try:
		ser.open()		# Opens the MultiWii serial port

	except Exception,e:	# catches any errors with opening serial ports
		print("Error open serial port: "+str(e))
		exit()
	
	if ser.isOpen(): #& ser2.isOpen():
		#time.sleep(13.2) 	# Exact time for Altitude data to become live.
		time.sleep(9)		# Gives time for the MultiWii to calibrate and begin sending live info
		print("Serial port is open at"+ser.portstr)

		file = open("data.csv", "w")
		#timestamp = time.clock()
		#multiwiiCMD()
		
		try:
			while True:
		#########################################################################
		# Sends and Recieves MSP commands then sends received data over TCP/IP
		#########################################################################

				ser.flushInput()	# cleans out the serial port
				ser.flushOutput()
				ser.write(MSP_ATTITUDE)	# sends MSP request
				time.sleep(0.02)	# gives adaquate time between MSP TX & RX
				timestamp = time.clock()
				response=ser.readline()	# reads MSP response
				ATTITUDE(response)	# sends to ATTITUDE to parse and update global variables
				ser.flushInput()	# cleans out the serial port
				ser.flushOutput()
				error = (time.clock() - timestamp)*10

				
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


				if beginFlag != 1:	# Won't send any data until both altitude and heading are valid data
					#print("A"+str(latitude)+","+str(longitude)+","+str(altitude)+","+str(heading)+","+str(timestamp)+"Z"+str(numSats)+","+str(accuracy)+","+str(pitch)+","+str(roll)+","+str(yaw)+","+str(throttle)+"W")	# print in CSV
					print(str(error)+" "+str(angx)+" "+str(angy)+" "+str(heading))	# print in CSV
					file.write(str(error)+","+str(angx)+","+str(angy)+","+str(heading)+"\n")
				else:			# If invalid, continue looping
					beginFlag = 0	# resets the flag
		       	ser.close()
		       	file.close()

	    	except Exception,e1:	# Catches any errors in the serial communication
	        	print("Error communicating..."+str(e1))
	else:
    		print("Cannot open serial port")

#-----------------------------------------------------------------------
if __name__=="__main__":
	main()