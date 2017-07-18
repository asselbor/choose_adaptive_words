from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QRect, Qt, QDate, QDateTime,pyqtSignal
from PyQt5.QtGui import QPainter, QColor, QFont, QBrush, QPen, QPixmap, QPalette

class ChildProfile(QtWidgets.QDialog):

	signal_profileCompleted = pyqtSignal()

	def __init__(self, parent=None):
		self.firstName = None
		self.lastName = None
		self.dateBirth = QDate(21, 07, 2001)
		self.rightHanded = True
		self.male = True
		self.section = 4

		super(ChildProfile, self).__init__(parent)
		uic.loadUi('../design/childProfile.ui', self)

		# connect slots
		self.e_firstName.textChanged.connect(self.firstNameChanged)
		self.e_lastName.textChanged.connect(self.lastNameChanged)
		self.e_birthDate.dateChanged.connect(self.birthDate_dateChanged)
		self.buttonBox.clicked.connect(self.profile_Complete)

		self.choice_class.addItem("Gd section")
		self.choice_class.addItem("CP")
		self.choice_class.addItem("CE1")
		self.choice_class.addItem("CE2")
		self.choice_class.addItem("CM1")
		self.choice_class.addItem("CM2")

		self.e_birthDate.setDate(QDate.currentDate())

		self.colorCompletedFrame()

		self.show()

	def colorCompletedFrame(self):
		# add red/green color of frame in function of the completed forms
		palette = QPalette()
		palette.setColor(self.backgroundRole(), QColor( 255, 102, 102))

		# set red
		self.c_lastname.setPalette(palette)
		self.c_lastname.setAutoFillBackground(True)

		self.c_firstname.setPalette(palette)
		self.c_firstname.setAutoFillBackground(True)

		self.c_birth.setPalette(palette)
		self.c_birth.setAutoFillBackground(True)

		# set green
		palette.setColor(self.backgroundRole(), QColor( 102, 255, 102))
		self.c_lateralite.setPalette(palette)
		self.c_lateralite.setAutoFillBackground( True)

		self.c_section.setPalette(palette)
		self.c_section.setAutoFillBackground(True)

		self.c_sex.setPalette(palette)
		self.c_sex.setAutoFillBackground(True)

	def profile_Complete(self):

		# complete profile
		self.firstName = self.e_firstName.toPlainText()
		self.lastName = self.e_lastName.toPlainText()
		self.dateBirth = self.e_birthDate.date()

		if self.S_male.isChecked():
			self.male = True
		else:
			self.male = False

		if self.L_rightHanded.isChecked():
			self.rightHanded = True
		else:
			self.rightHanded = False

		self.section = self.choice_class.currentIndex()
		self.signal_profileCompleted.emit()

	def isprofileCompleted(self):

		if(self.firstName != "" and self.lastName != "" and self.dateBirth < QDate.currentDate()):
			return True

		return False

	def firstNameChanged(self):
		palette = self.c_firstname.palette()

		if self.e_firstName.toPlainText() != "":
			palette.setColor(self.backgroundRole(), QColor( 102, 255, 102 ) )
			self.c_firstname.setPalette( palette )
			self.c_firstname.setAutoFillBackground( True )

		else:
			palette.setColor( self.backgroundRole(), QColor( 255, 102, 102) )
			self.c_firstname.setPalette( palette )
			self.c_firstname.setAutoFillBackground( True )
			

	def lastNameChanged(self):
		palette = self.c_firstname.palette()

		if self.e_lastName.toPlainText() != "":
			palette.setColor(self.backgroundRole(), QColor( 102, 255, 102 ) )
			self.c_lastname.setPalette( palette )
			self.c_lastname.setAutoFillBackground( True )

		else:
			palette.setColor( self.backgroundRole(), QColor( 255, 102, 102) )
			self.c_lastname.setPalette( palette )
			self.c_lastname.setAutoFillBackground( True )


	def birthDate_dateChanged(self):
		palette = self.c_birth.palette()

		if self.e_birthDate.date() < QDate.currentDate():
			palette.setColor(self.backgroundRole(), QColor( 102, 255, 102))
			self.c_birth.setPalette( palette )
			self.c_birth.setAutoFillBackground( True )
		
		else:
			palette.setColor( self.backgroundRole(), QColor( 255, 102, 102) )
			self.c_birth.setPalette( palette )
			self.c_birth.setAutoFillBackground( True )

