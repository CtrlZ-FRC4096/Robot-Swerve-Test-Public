
class CustomAnalog():
	""" This class provides a way to create a custom analog with any [float, int] function. """

	deadzone = 0
	inverted = False
	analogIn = None
	modifier = None

	def __call__(self):
		"""Calls get() on this CustomAnalog

		:returns: The value returned by get()
		"""
		if self.analogIn == None:
			raise RuntimeError("This object has not been initialized")
		return self.get()

	def __init__(self, analogIn):
		"""Initializes this CustomAnalog with a function to get the current analog value

		:param analogIn: A function that returns a [int,float] value when evaluated
		"""
		if not callable(analogIn):
			raise ValueError("The input passed into this custom analog is not a function!")

		self.analogIn = analogIn

	def get(self):
		"""Evaluates inverted, deadzone, and modifier on the current value of analogIn

		:returns: The evaluated value
		"""
		ret = self.analogIn()
		if abs(ret) < abs(self.deadzone):
			return 0
		if self.inverted == True:
			ret = -ret
		if self.modifier != None:
			ret = self.modifier(ret)
		return ret

	def getRaw(self):
		"""Gets the raw current value of analogIn

		:returns: the current value of analogIn without any evaluations done on it (raw)
		"""
		return self.analogIn()

	def setDeadzone(self, deadzone):
		"""Sets deadzone (distance from 0 to ignore)

		:param deadzone: deadzone amount
		"""
		self.deadzone = deadzone

	def setInverted(self, inverted = True):
		"""Sets inverted (whether the value of analogIn should be negated)

		:param inverted: invert? (default = true)
		"""
		self.inverted = inverted

	def setModifier(self, modifier) -> None:
		"""Sets the modifier (function that should be executed on analogIn)

		:param modifier: function that should be run
		"""
		self.modifier = modifier