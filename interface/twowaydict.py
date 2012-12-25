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
