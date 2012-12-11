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
driver['power_left']		= 0x01
driver['power_right']		= 0x02
driver['power_both']		= 0x05
driver['dir_left']			= 0x03
driver['dir_right']			= 0x04
driver['pid_toggle']		= 0x10
driver['speed_left']		= 0x11
driver['speed_right']		= 0x12
driver['speed_both']		= 0x15
driver['debug_off']			= 0x70
driver['debug_1']			= 0x71
driver['debug_2']			= 0x72
driver['debug_3']			= 0x73
driver['debug_4']			= 0x74
driver['debug_5']			= 0x75
driver['debug_6']			= 0x76
driver['debug_7']			= 0x77
driver['encoder_left']		= 0x41
driver['encoder_right']		= 0x21
driver['speed_act_left']	= 0x42
driver['speed_act_right']	= 0x22
driver['position_left']		= 0x44
driver['position_right']	= 0x24
driver['sensor_left']		= 0x46
driver['sensor_right']		= 0x26

mainCPU['feedback']			= '<'
mainCPU['base']				= 0x30 # OR this with other commands
mainCPU['arm']				= 0x40 # OR this with other commands
mainCPU['power_ccw_up']		= 0x00
mainCPU['power_cw_down']	= 0x01
mainCPU['encoder']			= 0x02
mainCPU['pid_p']			= 0x03
mainCPU['pid_i']			= 0x04
mainCPU['pid_d']			= 0x05
