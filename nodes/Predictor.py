import numpy as np
from array import array
from RNN_Model import Model
from datetime import datetime
import os
import sys
import pandas as pd
import pickle

import logging
logging.getLogger("tensorflow").setLevel(logging.WARNING)
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


class Predictor():
    def __init__(self):
        self.model = Model(200)
        with open('sizeLetters', 'rb') as fp:
            self.sizeLetters = pickle.load(fp)

    def codeChar(self, char):
        # digits from '1' to '9' get values from 1 to 9, and '0' gets a value 10
        # letters from 'a' to 'z' get values from 11 to 36
        if ord(char) > 96:
            return float(ord(char) - ord('a') + 11.0)
        elif ord(char) > 48:
            return float(ord(char) - ord('0'))
        else:
            return 10.0

    def interpolatePoint(self, db, a, ratio):
        x = db.x[a] + ratio*(db.x[a + 1] - db.x[a])
        y = db.y[a] + ratio*(db.y[a + 1] - db.y[a])
        t = db.time[a] + ratio*(db.time[a + 1] - db.time[a])
        return x, y, t

    def interpolate(self, db, index, size):
        a = np.floor((len(db)-1)/(size-1)*index)
        ratio = (len(db)-1)/(size-1)*index - a
        return self.interpolatePoint(db, a, ratio)

    def getTimeLengthForSqueezedTime(self, db, threshold):
        substract = 0
        for i in range(1, len(db) - 1):
            substract = substract + max(db.time[i] - db.time[i-1] - threshold, 0)
        return db.time[len(db) - 1] - substract

    def makeItFlat(self, db, size):
        x = []
        y = []
        t = []
        xp = db.x[0]
        yp = db.y[0]
        tp = db.time[0]
        for i in range(1, size - 1):
            xc, yc, tc = self.interpolate(db, i, size)
            x.append(xc - xp)
            y.append(yc - yp)
            t.append(tc - tp)
            xp, yp, tp = xc, yc, tc
            
        x.append(db.x[len(db) - 1] - xp)
        y.append(db.y[len(db) - 1] - yp)
        t.append(db.time[len(db) - 1] - tp)
        
        return pd.DataFrame({'x': x, 'y': y, 't': t, 
                             'gender': db.gender[0], 'laterality': db.laterality[0],
                             'grade': db.grade[0]},
                           columns=['x', 'y', 't', 'gender', 'laterality', 'grade', 'id'])

    def normalize(self, data):
        data.x /= 60
        data.x -= 0.2
        data.x = data.x.clip(-15, 15)
        data.y /= 60
        data.y = data.y.clip(-15, 15)
        data.t /= 41
        data.t = data.t.clip(0, 15)
        data.t -= 1
        data.grade /= 6
        data.grade -= 0.5
        data.gender -= 0.5
        data.laterality -= 0.5

    def predict(self, laterality, gender, ageMonth, grade, data, glyphWritten):

        # create db
        db = self.createDF(laterality, gender, ageMonth, grade, data)
        db = self.scaleDb(db, glyphWritten)

        softmax = 0

        # Remove doublons
        prime = 23
        nonDoublons = [True]
        nonDoublons.extend((db.x[1:].values != db.x[:-1].values) | (db.y[1:].values != db.y[:-1].values))
        db = db[nonDoublons].reset_index(drop = True)

        diff = self.makeItFlat(db, int(self.getTimeLengthForSqueezedTime(db, 250)/prime))
        self.normalize(diff)

        # rnn seq data
        x = np.transpose([diff.x.values[:-1], diff.y.values[:-1], 
            diff.t.values[:-1], diff.gender.values[:-1], 
            diff.grade.values[:-1], diff.laterality.values[:-1]]).tolist()

        # check seqlen
        seqLen = len(x)

        # 0 padding
        while(len(x) < self.model.max_length):
            x.append([0.,0.,0.,0.,0.,0.])
        if len(x) >= self.model.max_length:
            x = x[0:self.model.max_length]

        # predict
        try:
            softmax = self.model.predict([x], seqLen)[0]
        except:
            pass

        return softmax[int(self.codeChar(glyphWritten))]

    def createDF(self, laterality, gender, ageMonth, grade, data):

        df = pd.DataFrame()

        df["time"] = [d.time for d in data]
        df["time"] = df["time"] - np.min(df["time"])

        df["x"] = [d.x for d in data]
        df["x"] = df["x"] - np.min(df["x"])

        df["y"] = [-d.y for d in data]
        df["y"] = df["y"] - np.min(df["y"])
        
        df["x_tilt"] = [d.x_tilt for d in data]
        df["y_tilt"] = [d.y_tilt for d in data]
        df["pressure"] = [d.pressure for d in data]

        df["laterality"] = float(laterality)
        df["gender"] = float(gender)
        df["ageMonth"] = float(ageMonth)
        df["grade"] = float(grade)

        return df

    def scaleDb(self, db, glyphWritten):

        scaleX = (max(db.x.values) - min(db.x.values))/float(self.sizeLetters[glyphWritten][0])
        scaleY = (max(db.y.values) - min(db.y.values))/float(self.sizeLetters[glyphWritten][1])

        scale = (scaleX + scaleY)/2.

        db["x"] = db["x"] / scale
        db["y"] = db["y"] / scale

        return db





