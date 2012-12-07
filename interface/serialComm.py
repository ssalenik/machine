#!/bin/python

"""
contains definitions for serial communication delimiters, commands, etc
"""
# new line, always ends all commands and requests
NL = '\n'
FEEDBACK = '>'
DRIVER = 'p'

class SerialComm:

	def __init__(self, parent=None):

		# init values
		# NOTE : these are the actual values, not their corresponding serial codes
		# we'll use this module to store the values for now
		# TODO: decide whether there should be a serialComm class or not

		# driver init values
		self.power_left = 0
		self.power_right = 0
		self.power_both = 0
		self.dir_left = 0
		self.dir_right = 0
		self.pid_toggle = False
		self.speed_left = 0
		self.speed_right = 0
		self.speed_both= 0

		# feedback init values
		self.refresh_all = 0
		self.encoder_left = 0
		self.encoder_right = 0
		self.sensor_left = 0
		self.sensor_right = 0
		self.encoder_base = 0
		self.sensor_base = 0
		self.encoder_arm = 0
		self.encoder_claw = 0
		self.encoder_claw_height = 0


# serial send codes start here



power_left = 23

def powerLeft(value=None):
	power_left = 23
	if value :
		power_left = value
	else :
		return hex(power_left)

#dictionaries
commands = TwoWayDict()
feedback = TwoWayDict()

commands[powerLeft] = 0x01



# commands = {
# 	power_left : 0x01,
# 	power_right : 0x02,
# 	power_both : 0x05,
# 	dir_left : 0x03,
# 	dir_right : 0x04,
# 	pid_toggle : 0x10,
# 	speed_left : 0x11,
# 	speed_right : 0x12,
# 	speed_both : 0x15
# 	# there are more...
# }

# # feedback
# feedback = {
# 	refresh_all : 0x00,
# 	encoder_left : 0x01,
# 	encoder_right: 0x02,
# 	sensor_left : 0x11,
# 	sensor_right : 0x12,
# 	encoder_base : 0x20,
# 	sensor_base : 0x21,
# 	encoder_arm : 0x22,
# 	encoder_claw : 0x30,
# 	encoder_claw_height : 0x31
# 	# add more as needed
# }

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