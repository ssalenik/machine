#!/bin/python
from twowaydict import *
from struct import *

"""
contains definitions for serial communication delimiters, commands, etc
"""

# new line, always ends all commands and requests
EOL = '\r'
STOP_ALL = ' '
FORWARD = 0
BACKWARD = 1
# not sure about these
CW = 0
CCW = 1

driver = TwoWayDict()
mainCPU = TwoWayDict()

driver['cmd']				= 'p'
driver['feedback']			= '>'
driver['set_power_left']	= 0x01
driver['set_power_right']	= 0x02
driver['set_power_both']	= 0x05
driver['set_dir_left']		= 0x03
driver['set_dir_right']		= 0x04
driver['pid_toggle']		= 0x10
driver['set_speed_left']	= 0x11
driver['set_speed_right']	= 0x12
driver['set_speed_both']	= 0x15
driver['debug_off']			= 0x70
driver['debug_1']			= 0x71
driver['debug_2']			= 0x72
driver['debug_3']			= 0x73
driver['debug_4']			= 0x74
driver['debug_5']			= 0x75
driver['debug_6']			= 0x76
driver['debug_7']			= 0x77
driver['dir_power_both']	= 0x40
driver['encoder_both']		= 0x41
driver['speed_both']		= 0x42
driver['acc_both']			= 0x43
driver['abs_pos_both']		= 0x44
driver['rel_pos_both']		= 0x45
driver['sensor_both']		= 0x46
driver['pos_err_both']		= 0x47

mainCPU['feedback']			= '<'
mainCPU['base']				= 0x30 # OR this with other commands
mainCPU['arm']				= 0x40 # OR this with other commands
mainCPU['arm_pid_off']		= 0x10
mainCPU['arm_pid_on']		= 0x11 
mainCPU['base_power_up']	= 0x30
mainCPU['base_power_down']	= 0x31
mainCPU['base_encoder']		= 0x32
mainCPU['base_pid_s']		= 0x33
mainCPU['base_pid_p']		= 0x34
mainCPU['base_pid_i']		= 0x35
mainCPU['base_pid_d']		= 0x36
mainCPU['arm_power_up']		= 0x40
mainCPU['arm_power_down']	= 0x41
mainCPU['arm_encoder']		= 0x42
mainCPU['arm_pid_s']		= 0x43
mainCPU['arm_pid_p']		= 0x44
mainCPU['arm_pid_i']		= 0x45
mainCPU['arm_pid_d']		= 0x46
mainCPU['ping_pong_servo']	= 0x50
mainCPU['laser_servo']		= 0x60
mainCPU['claw_servo_1']		= 0x70  # upward/downward
mainCPU['claw_servo_2']		= 0x80  # open/close
mainCPU['magnets_off']		= 0x90
mainCPU['magnets_on']		= 0x91

driver_decode = {
	'dir_power_both'	: '!BBB',
	'encoder_both'		: '!ii',
	'speed_both'		: '!hh',
	'acc_both'			: '!hh',
	'abs_pos_both'		: '!hh',
	'rel_pos_both'		: '!BhBh',
	'sensor_both'		: '!BB',
	'pos_err_both'		: '!ii'
}

mainCPU_decode = {
	'base_encoder'		: '!h',
	'base_pid_p'		: '!h',
	'base_pid_i'		: '!i',
	'base_pid_d'		: '!h',
	'base_pid_s'		: '!h',
	'arm_encoder'		: '!h',
	'arm_pid_p'			: '!h',
	'arm_pid_i'			: '!i',
	'arm_pid_d'			: '!h',
	'arm_pid_s'			: '!h'
}

# old way:
# mainCPU['feedback']		= '<'
# mainCPU['base']			= 0x30 # OR this with other commands
# mainCPU['arm']			= 0x40 # OR this with other commands
# mainCPU['power_ccw_up']	= 0x00
# mainCPU['power_cw_down']	= 0x01
# mainCPU['encoder']		= 0x02
# mainCPU['pid_p']			= 0x03
# mainCPU['pid_i']			= 0x04
# mainCPU['pid_d']			= 0x05

def getDataDecode(prepend, code):
	"""
	input: the prepend and code string
	returns: str which is the first parameter for the unpack struct function (eg: '!b' for a signed byte)
	returns empty str if it could not decode the command
	"""
	unpackString = ""

	# for now code has to be 2 chars
	if len(code) < 2 :
		return None

	try:
		hex_code  = ord((code.decode('hex'))) #converts to int
	except:
		# non hex digit in code
		return None

	# make sure prepend and code are recognized at all, first of all
	if not ((prepend in driver and hex_code in driver) or (prepend in mainCPU and hex_code in mainCPU)):
		return None

	if prepend == driver['feedback'] :
		unpackString = driver_decode[driver[hex_code]]

	elif prepend == mainCPU['feedback'] :
		unpackString = mainCPU_decode[mainCPU[hex_code]]

	return unpackString

def getData(prepend, code, data):
	"""
	input: the prepend, code, and data strings
	returns: list of data decoded to integers
	empty list if decoding was not possible
	"""
	decodedData = []

	decode = getDataDecode(prepend, code)

	# make sure we know the decoding, else return
	if not decode :
		return None

	try:
		hexValue = data.decode('hex')	# turn str in hex str
		decodedData =  unpack(decode, hexValue)
	except :
		None

	return decodedData

def isValidMessage(prepend, code, data):
	"""
	returns True if message is valid, False otherwise
	checks by tryig to decode the data
	"""

	if getData(prepend, code, data):
		return True
	else :
		return False
