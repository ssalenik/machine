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
FORWARD = 0
BACKWARD = 1
# not sure about these
CW = 0
CCW = 1

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
commands['debug_off']	= 0x70
commands['debug_1']		= 0x71
commands['debug_2']		= 0x72
commands['debug_3']		= 0x73
commands['debug_4']		= 0x74
commands['debug_5']		= 0x75
commands['debug_6']		= 0x76
commands['debug_7']		= 0x77

# there are more...


# feedback
feedback['delim_driver']		= '>'
feedback['delim_main']			= '<'
# feedback['refresh_all']			=
feedback['encoder_left']		= 0x41
feedback['encoder_right']		= 0x21
# feedback['encoder_both']			= 
feedback['speed_act_left']		= 0x42
feedback['speed_act_right']		= 0x22
# feedback['speed_both']			= 
feedback['position_left']		= 0x44
feedback['position_right']		= 0x24
# feedback['position_both']			= 
feedback['sensor_left']			= 0x46
feedback['sensor_right']		= 0x26
# feedback['sensor_both']			= 
# feedback['encoder_base']			= 
# feedback['sensor_base']			= 
# feedback['encoder_arm']			= 
# feedback['encoder_claw']			= 
# feedback['encoder_claw_height']	= 

# add more as needed