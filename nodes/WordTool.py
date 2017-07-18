import pickle
from collections import Counter
import re

class WordTool():

	def __init__(self):

		self.letters = []
		with open('child_dico.txt') as fin:
			lines = (word.strip().upper() for word in fin)
			self.easy_words = [(word, Counter(word)) for word in lines  if not re.match(r'.*[\%\$\^\*\@\!\_\-\(\)\:\;\'\"\{\}\[\]].*', word) and len(word) < 8 and len(word) >= 2]

		with open('/usr/share/dict/words') as fin:
			lines = (word.strip().upper() for word in fin)
			self.words = [(word, Counter(word)) for word in lines  if not re.match(r'.*[\%\$\^\*\@\!\_\-\(\)\:\;\'\"\{\}\[\]].*', word) and len(word) < 8 and len(word) >= 2]


	def separateWordsToLetters(self, word, boxes, height, pix_to_meter):

		letters = {}
		for i in range(len(boxes)):
			letters[i] = []


		for data in word:
			for i, box in enumerate(boxes):

				x =  data.x*pix_to_meter
				y = -data.y*pix_to_meter + height*pix_to_meter

				if x >= box[0] and x < box[2] and y >= box[1] and y < box[3]:
					letters[i].append(data)

		return letters


	def findWordWithLetters(self, letters, childMode=True):

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

