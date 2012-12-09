#!/bin/python

"""
contains definitions for serial communication delimiters, commands, etc
"""

class TwoWayDict(dict):
	"""
	two way dictionary class taken from http://stackoverflow.com/questions/1456373/two-way-reverse-map

	this way we can look up the code and what its for
	"""

	def __len__(self):
		return dict.__len__(self) / 2

	def __setitem__(self, key, value):
		dict.__setitem__(self, key, value)
		dict.__setitem__(self, value, key)

# new line, always ends all commands and requests
EOL = '\r'

commands = TwoWayDict()
feedback = TwoWayDict()

commands['driver']		= 'p'
commands['stop_all']	= ' '
commands['power_left']	= 0x01
commands['power_right']	= 0x02
commands['power_both']	= 0x05
commands['dir_left']	= 0x03
commands['dir_right']	= 0x04
commands['pid_toggle']	= 0x10
commands['speed_left']	= 0x11
commands['speed_right']	= 0x12
commands['speed_both']	= 0x15

# there are more...


# feedback
feedback['delim']				= '>'
feedback['refresh_all']			= 0x00
feedback['encoder_left']		= 0x01
feedback['encoder_right']		= 0x02
feedback['encoder_both']		= 0x03
feedback['speed_act_left']		= 0x04
feedback['speed_act_right']		= 0x05
feedback['speed_both']			= 0x06
feedback['position_left']		= 0x07
feedback['position_right']		= 0x08
feedback['position_both']		= 0x09
feedback['sensor_left']			= 0x10
feedback['sensor_right']		= 0x11
feedback['sensor_both']			= 0x12
feedback['encoder_base']		= 0x20
feedback['sensor_base']			= 0x21
feedback['encoder_arm']			= 0x22
feedback['encoder_claw']		= 0x30
feedback['encoder_claw_height']	= 0x31

# add more as needed


# class SerialComm:

# 	def __init__(self, parent=None):

# 		# init values
# 		# NOTE : these are the actual values, not their corresponding serial codes
# 		# we'll use this module to store the values for now
# 		# TODO: decide whether there should be a serialComm class or not

# 		# driver init values
# 		self.power_left = 0
# 		self.power_right = 0
# 		self.power_both = 0
# 		self.dir_left = 0
# 		self.dir_right = 0
# 		self.pid_toggle = False
# 		self.speed_left = 0
# 		self.speed_right = 0
# 		self.speed_both= 0

# 		# feedback init values
# 		self.refresh_all = 0
# 		self.encoder_left = 0
# 		self.encoder_right = 0
# 		self.sensor_left = 0
# 		self.sensor_right = 0
# 		self.encoder_base = 0
# 		self.sensor_base = 0
# 		self.encoder_arm = 0
# 		self.encoder_claw = 0
# 		self.encoder_claw_height = 0


# # serial send codes start here



# power_left = 23

# def powerLeft(value=None):
# 	power_left = 23
# 	if value :
# 		power_left = value
# 	else :
# 		return hex(power_left)

# #dictionaries
# commands = TwoWayDict()
# feedback = TwoWayDict()

# commands[powerLeft] = 0x01





