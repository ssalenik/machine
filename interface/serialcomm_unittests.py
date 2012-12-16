"""
These are a series of unit tests to make sure the data encoding and decoding works properly
"""

import unittest
from struct import *

# local source imports
from serialcomm import *
from controller import *
from logger import *

class TestMessageParsing(unittest.TestCase):
	
	def setUp(self):
		self.logger = Logger(output = self.output)
		self.controller = Controller(output = self.output, logger=self.logger)

	def test_compose_base_pid_i(self):
		data = -43567
		message_expected = "34"
		message_expected += pack("!i", data).encode('hex')
		message_expected = message_expected.upper()
		message_expected += "\r"

		print ""
		print "base pid i value:\t%i" % data
		print "expected message:\t%s" % message_expected.rstrip()

		message = self.controller.composeMessage(code=mainCPU['base_pid_i'], data=data, sendToDriver=False, encoding='s32')
		message += '\r'

		print "composed message:\t%s" % message.rstrip()

		self.assertEqual(message, message_expected)

		message = "<" + message

		print "sending message:\t%s" % message.rstrip()

		self.controller.parseMessage(message)

		self.assertEqual(data, self.controller.i_base)

		print "received pid i value:\t%i" % self.controller.i_base

	def test_compose_arm_encoder(self):
		data = -1
		message_expected = "42"
		message_expected += pack("!h", data).encode('hex')
		message_expected = message_expected.upper()
		message_expected += "\r"

		print ""
		print "arm encoder value:\t%i" % data
		print "expected message:\t%s" % message_expected.rstrip()

		message = self.controller.composeMessage(code=mainCPU['arm_encoder'], data=data, sendToDriver=False, encoding='s16')
		message += '\r'

		print "composed message:\t%s" % message.rstrip()

		self.assertEqual(message, message_expected)

		message = "<" + message

		print "sending message:\t%s" % message.rstrip()

		self.controller.parseMessage(message)

		self.assertEqual(data, self.controller.encoder_arm)

		print "received arm encoder value:\t%i" % self.controller.encoder_arm

	def test_receive_encodervalue(self):
		data = [0, -23451]
		message = ">41"
		message += pack("!ii", data[0], data[1]).encode('hex')
		message += "\r\n"

		self.controller.parseMessage(message)

		print ""
		print "actual sens encoder:\t%i\t%i" % (data[0], data[1])
		print "sending message:\t%s" % message.rstrip()
		print "received sens encoder:\t%i\t%i" % (self.controller.encoder_left, self.controller.encoder_right)

		self.assertEqual(self.controller.encoder_left, data[0])
		self.assertEqual(self.controller.encoder_right, data[1])

	def test_receive_max_poserrvalue(self):
		data = [-2147483648, 2147483647]
		message = ">47"
		message += pack("!ii", data[0], data[1]).encode('hex')
		message += "\r\n"

		self.controller.parseMessage(message)

		print ""
		print "actual pos error:\t%i\t%i" % (data[0], data[1])
		print "sending message:\t%s" % message.rstrip()
		print "received pos error:\t%i\t%i" % (self.controller.pos_err_left, self.controller.pos_err_right)

		self.assertEqual(self.controller.pos_err_left, data[0])
		self.assertEqual(self.controller.pos_err_right, data[1])

	def output(self, text):
		print text

if __name__ == '__main__':
	unittest.main()