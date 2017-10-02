import numpy as np
from parameters import *

class Glyph():

	def __init__(self, glyph):
		self.glyph = glyph
		self.dScore = [0.5]
		self.nbTime = 0
		self.iterationOccurence = []

	def getUncertainty(self):
		if len(self.dScore) == 1:
			return 1
		elif len(self.dScore) <= NB_LETTERS_UNCERTAINTY_COMPUTATION:
			return np.std(self.dScore)
		else:
			return np.std(self.dScore[-NB_LETTERS_UNCERTAINTY_COMPUTATION:])

	def getMastery(self):
		# do a weighted average do give more importance to last values
		if len(self.dScore) > 1:
			return np.average(np.array(self.dScore[1:]), weights=[(i + 1)**2 for i in range(len(self.dScore[1:]))])
		else:
			return self.dScore[-1]


	def getProgress(self):
		if len(self.dScore) <= 1:
			return 0
		else:
			x = np.array(range(len(self.dScore)))
			z = np.polyfit(x, self.dScore, POLYNOMIAL_DEGREE_POLYFIT)
			return z[0]
