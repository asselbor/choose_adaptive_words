import pickle
import re
import pandas as pd
import math

from collections import Counter

class WordTool():

	def __init__(self):

		self.words = {}
		self.df_req = pd.read_csv("requirement.csv")


		# create lsit of words
		with open('child_dico.txt') as fin:    #open('/usr/share/dict/words')
			lines = (word.strip().upper() for word in fin)

			for word in lines:
				if not re.match(r'.*[\%\$\^\*\@\!\_\-\(\)\:\;\'\"\{\}\[\]].*', word) and len(word) < 8 and len(word) >= 3 and " " not in word:
					self.words[word.lower()] = {"pertinence": 0, "mastery": 0, "uncertainty": 0}

	def separateWordsToLetters(self, word, boxes, height, pix_to_meter, absToRel):

		letters = {}
		for i in range(len(boxes)):
			letters[i] = []


		for data in word:
			for i, box in enumerate(boxes):

				x =  data.x*pix_to_meter - absToRel.x()*pix_to_meter
				y = -(data.y*pix_to_meter - absToRel.y()*pix_to_meter) + height*pix_to_meter 
				if x >= box[0] and x < box[2] and y >= box[1] and y < box[3]:
					letters[i].append(data)

		return letters

	def findWordWithLetters(self, letters, childMode=True): # deprecated

		rack = Counter(letters.upper())

		if childMode:
			for scrabble_word, letter_count in self.easy_words:
				# Using length here to limit output for example purposes
				if len(scrabble_word) >= len(letters) and not (rack - letter_count):
					return(scrabble_word)
		else:
			for scrabble_word, letter_count in self.words:
				# Using length here to limit output for example purposes
				if len(scrabble_word) >= len(letters) and not (rack - letter_count):
					return(scrabble_word)

	def activationFct(self, x):
		x = (x - .5)*3.
		fact = 1.2

		return ((math.exp(fact*x) - math.exp(-fact*x))/(math.exp(fact*x)+math.exp(-fact*x)) + 1)/2.

	def updateWords(self, glyphs):

		for word in self.words:
			self.words[word]["pertinence"] = self.computePertinence(glyphs, word)
			self.words[word]["mastery"] = self.computeMastery(glyphs, word)
			self.words[word]["uncertainty"] = self.computeUncertainty(glyphs, word)

	def computePertinence(self, glyphs, word):

		pertinence = 0
		for letter in word:
			if letter in self.df_req.letter.unique():
				pertinence += self.activationFct(glyphs[letter].getMastery())
			else:
				pertinence += 1


		return pertinence / float(len(word))

	def computeMastery(self, glyphs, word):

		mastery = 0
		for letter in word:
			mastery += glyphs[letter].getMastery()

		return (mastery/float(len(word)))

	def computeUncertainty(self, glyphs, word):

		uncertainty = 0
		for letter in word:
			uncertainty += glyphs[letter].getUncertainty()

		return uncertainty / float(len(word))
