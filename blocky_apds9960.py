from micropython import const
APDS9960_I2C_ADDR = const(0x39)
APDS9960_GESTURE_THRESHOLD_OUT = const(10)
APDS9960_GESTURE_SENSITIVITY_1 = const(50)
APDS9960_GESTURE_SENSITIVITY_2 = const(20)
APDS9960_DEV_ID = [0xab, 0x9c, 0xa8]
APDS9960_TIME_FIFO_PAUSE = const(300)
APDS9960_REG_ENABLE = const(0x80)
APDS9960_REG_ATIME = const(0x81)
APDS9960_REG_WTIME = const(0x83)
APDS9960_REG_AILTL = const(0x84)
APDS9960_REG_AILTH = const(0x85)
APDS9960_REG_AIHTL = const(0x86)
APDS9960_REG_AIHTH = const(0x87)
APDS9960_REG_PILT = const(0x89)
APDS9960_REG_PIHT = const(0x8b)
APDS9960_REG_PERS = const(0x8c)
APDS9960_REG_CONFIG1 = const(0x8d)
APDS9960_REG_PPULSE = const(0x8e)
APDS9960_REG_CONTROL = const(0x8f)
APDS9960_REG_CONFIG2 = const(0x90)
APDS9960_REG_ID = const(0x92)
APDS9960_REG_STATUS = const(0x93)
APDS9960_REG_CDATAL = const(0x94)
APDS9960_REG_CDATAH = const(0x95)
APDS9960_REG_RDATAL = const(0x96)
APDS9960_REG_RDATAH = const(0x97)
APDS9960_REG_GDATAL = const(0x98)
APDS9960_REG_GDATAH = const(0x99)
APDS9960_REG_BDATAL = const(0x9a)
APDS9960_REG_BDATAH = const(0x9b)
APDS9960_REG_PDATA = const(0x9c)
APDS9960_REG_POFFSET_UR = const(0x9d)
APDS9960_REG_POFFSET_DL = const(0x9e)
APDS9960_REG_CONFIG3 = const(0x9f)
APDS9960_REG_GPENTH = const(0xa0)
APDS9960_REG_GEXTH = const(0xa1)
APDS9960_REG_GCONF1 = const(0xa2)
APDS9960_REG_GCONF2 = const(0xa3)
APDS9960_REG_GOFFSET_U = const(0xa4)
APDS9960_REG_GOFFSET_D = const(0xa5)
APDS9960_REG_GOFFSET_L = const(0xa7)
APDS9960_REG_GOFFSET_R = const(0xa9)
APDS9960_REG_GPULSE = const(0xa6)
APDS9960_REG_GCONF3 = const(0xaA)
APDS9960_REG_GCONF4 = const(0xaB)
APDS9960_REG_GFLVL = const(0xae)
APDS9960_REG_GSTATUS = const(0xaf)
APDS9960_REG_IFORCE = const(0xe4)
APDS9960_REG_PICLEAR = const(0xe5)
APDS9960_REG_CICLEAR = const(0xe6)
APDS9960_REG_AICLEAR = const(0xe7)
APDS9960_REG_GFIFO_U = const(0xfc)
APDS9960_REG_GFIFO_D = const(0xfd)
APDS9960_REG_GFIFO_L = const(0xfe)
APDS9960_REG_GFIFO_R = const(0xff)
APDS9960_BIT_PON = const(0b00000001)
APDS9960_BIT_AEN = const(0b00000010)
APDS9960_BIT_PEN = const(0b00000100)
APDS9960_BIT_WEN = const(0b00001000)
APSD9960_BIT_AIEN =const(0b00010000)
APDS9960_BIT_PIEN = const(0b00100000)
APDS9960_BIT_GEN = const(0b01000000)
APDS9960_BIT_GVALID = const(0b00000001)
APDS9960_MODE_POWER = const(0)
APDS9960_MODE_AMBIENT_LIGHT = const(1)
APDS9960_MODE_PROXIMITY = const(2)
APDS9960_MODE_WAIT = const(3)
APDS9960_MODE_AMBIENT_LIGHT_INT = const(4)
APDS9960_MODE_PROXIMITY_INT = const(5)
APDS9960_MODE_GESTURE = const(6)
APDS9960_MODE_ALL = const(7)
APDS9960_LED_DRIVE_100MA = const(0)
APDS9960_LED_DRIVE_50MA = const(1)
APDS9960_LED_DRIVE_25MA = const(2)
APDS9960_LED_DRIVE_12_5MA = const(3)
APDS9960_PGAIN_1X = const(0)
APDS9960_PGAIN_2X = const(1)
APDS9960_PGAIN_4X = const(2)
APDS9960_PGAIN_8X = const(3)
APDS9960_AGAIN_1X = const(0)
APDS9960_AGAIN_4X = const(1)
APDS9960_AGAIN_16X = const(2)
APDS9960_AGAIN_64X = const(3)
APDS9960_GGAIN_1X = const(0)
APDS9960_GGAIN_2X = const(1)
APDS9960_GGAIN_4X = const(2)
APDS9960_GGAIN_8X = const(3)
APDS9960_LED_BOOST_100 = const(0)
APDS9960_LED_BOOST_150 = const(1)
APDS9960_LED_BOOST_200 = const(2)
APDS9960_LED_BOOST_300 = const(3)
APDS9960_GWTIME_0MS = const(0)
APDS9960_GWTIME_2_8MS = const(1)
APDS9960_GWTIME_5_6MS = const(2)
APDS9960_GWTIME_8_4MS = const(3)
APDS9960_GWTIME_14_0MS = const(4)
APDS9960_GWTIME_22_4MS = const(5)
APDS9960_GWTIME_30_8MS = const(6)
APDS9960_GWTIME_39_2MS = const(7)
APDS9960_DEFAULT_ATIME = const(219)
APDS9960_DEFAULT_WTIME = const(246)
APDS9960_DEFAULT_PROX_PPULSE = const(0x87)
APDS9960_DEFAULT_GESTURE_PPULSE = const(0x89)
APDS9960_DEFAULT_POFFSET_UR = const(0)
APDS9960_DEFAULT_POFFSET_DL = const(0)
APDS9960_DEFAULT_CONFIG1 = const(0x60)
APDS9960_DEFAULT_LDRIVE = APDS9960_LED_DRIVE_100MA
APDS9960_DEFAULT_PGAIN = APDS9960_PGAIN_4X
APDS9960_DEFAULT_AGAIN = APDS9960_AGAIN_4X
APDS9960_DEFAULT_PILT = const(0)
APDS9960_DEFAULT_PIHT = const(50)
APDS9960_DEFAULT_AILT = const(0xffff) 
APDS9960_DEFAULT_AIHT = const(0)
APDS9960_DEFAULT_PERS = const(0x11)
APDS9960_DEFAULT_CONFIG2 = const(0x01)
APDS9960_DEFAULT_CONFIG3 = const(0)
APDS9960_DEFAULT_GPENTH = const(40)
APDS9960_DEFAULT_GEXTH = const(30)
APDS9960_DEFAULT_GCONF1 = const(0x40)
APDS9960_DEFAULT_GGAIN = APDS9960_GGAIN_4X
APDS9960_DEFAULT_GLDRIVE = APDS9960_LED_DRIVE_100MA
APDS9960_DEFAULT_GWTIME = APDS9960_GWTIME_2_8MS
APDS9960_DEFAULT_GOFFSET = const(0)
APDS9960_DEFAULT_GPULSE = const(0xc9)
APDS9960_DEFAULT_GCONF3 = const(0)
APDS9960_DEFAULT_GIEN = const(0)
APDS9960_DIR_NONE = const(0)
APDS9960_DIR_LEFT = const(1)
APDS9960_DIR_RIGHT = const(2)
APDS9960_DIR_UP = const(3)
APDS9960_DIR_DOWN = const(4)
APDS9960_DIR_NEAR = const(5)
APDS9960_DIR_FAR = const(6)
APDS9960_DIR_ALL = const(7)
APDS9960_STATE_NA = const(0)
APDS9960_STATE_NEAR = const(1)
APDS9960_STATE_FAR = const(2)
APDS9960_STATE_ALL = const(3)

#============================
from const import *
from exceptions import *
from time import sleep_ms as sleep 
from machine import I2C , Pin 
from blocky_hardware import *

class APDS9960:
	class GestureData:
		def __init__(self):
			self.u_data = [0] * 32
			self.d_data = [0] * 32
			self.l_data = [0] * 32
			self.r_data = [0] * 32
			self.index = 0
			self.total_gestures = 0
			self.in_threshold = 0
			self.out_threshold = 0

	def __init__(self, PortName, address=APDS9960_I2C_ADDR, valid_id=APDS9960_DEV_ID):
		# I2C stuff
		self.address = address
		self.bus = I2C( scl = Pin(PortName["pin1"]) , sda = Pin(PortName["pin2"]))
		

		# instance variables for gesture detection
		self.gestue_ud_delta_ = 0
		self.gesture_lr_delta_ = 0
	
		self.gesture_ud_count_ = 0
		self.gesture_lr_count_ = 0
	
		self.gesture_near_count_ = 0
		self.gesture_far_count_ = 0
	
		self.gesture_state_ = 0
		self.gesture_motion_ = APDS9960_DIR_NONE

		self.gesture_data_ = APDS9960.GestureData()
	
		# check device id
		self.dev_id = self._read_byte_data(APDS9960_REG_ID)
		if not self.dev_id in valid_id:
			print("NotValid")
			
		# disable all features
		self.setMode(APDS9960_MODE_ALL, False)

		# set default values for ambient light and proximity registers
		self._write_byte_data(APDS9960_REG_ATIME, APDS9960_DEFAULT_ATIME)
		self._write_byte_data(APDS9960_REG_WTIME, APDS9960_DEFAULT_WTIME)
		self._write_byte_data(APDS9960_REG_PPULSE, APDS9960_DEFAULT_PROX_PPULSE)
		self._write_byte_data(APDS9960_REG_POFFSET_UR, APDS9960_DEFAULT_POFFSET_UR)
		self._write_byte_data(APDS9960_REG_POFFSET_DL, APDS9960_DEFAULT_POFFSET_DL)
		self._write_byte_data(APDS9960_REG_CONFIG1, APDS9960_DEFAULT_CONFIG1)
		self.setLEDDrive(APDS9960_DEFAULT_LDRIVE)
		self.setProximityGain(APDS9960_DEFAULT_PGAIN)
		self.setAmbientLightGain(APDS9960_DEFAULT_AGAIN)
		self.setProxIntLowThresh(APDS9960_DEFAULT_PILT)
		self.setProxIntHighThresh(APDS9960_DEFAULT_PIHT)
		self.setLightIntLowThreshold(APDS9960_DEFAULT_AILT)
		self.setLightIntHighThreshold(APDS9960_DEFAULT_AIHT)
		
		self._write_byte_data(APDS9960_REG_PERS, APDS9960_DEFAULT_PERS)
		self._write_byte_data(APDS9960_REG_CONFIG2, APDS9960_DEFAULT_CONFIG2)
		self._write_byte_data(APDS9960_REG_CONFIG3, APDS9960_DEFAULT_CONFIG3)
	
		# set default values for gesture sense registers
		self.setGestureEnterThresh(APDS9960_DEFAULT_GPENTH)
		self.setGestureExitThresh(APDS9960_DEFAULT_GEXTH)
		self._write_byte_data(APDS9960_REG_GCONF1, APDS9960_DEFAULT_GCONF1)

		self.setGestureGain(APDS9960_DEFAULT_GGAIN)
		self.setGestureLEDDrive(APDS9960_DEFAULT_GLDRIVE)
		self.setGestureWaitTime(APDS9960_DEFAULT_GWTIME)
		self._write_byte_data(APDS9960_REG_GOFFSET_U, APDS9960_DEFAULT_GOFFSET)
		self._write_byte_data(APDS9960_REG_GOFFSET_D, APDS9960_DEFAULT_GOFFSET)
		self._write_byte_data(APDS9960_REG_GOFFSET_L, APDS9960_DEFAULT_GOFFSET)
		self._write_byte_data(APDS9960_REG_GOFFSET_R, APDS9960_DEFAULT_GOFFSET)
		self._write_byte_data(APDS9960_REG_GPULSE, APDS9960_DEFAULT_GPULSE)
		self._write_byte_data(APDS9960_REG_GCONF3, APDS9960_DEFAULT_GCONF3)
		self.setGestureIntEnable(APDS9960_DEFAULT_GIEN)


	def getMode(self):
		return self._read_byte_data(APDS9960_REG_ENABLE)
	
	def setMode(self, mode, enable=True):
		# read ENABLE register
		reg_val = self.getMode()
	
		if mode < 0 or mode > APDS9960_MODE_ALL:
			raise ADPS9960InvalidMode(mode)
	
		# change bit(s) in ENABLE register */
		if mode == APDS9960_MODE_ALL:
			if enable:
				reg_val = 0x7f
			else:
				reg_val = 0x00
		else:
			if enable:
				reg_val |= (1 << mode);
			else:
				reg_val &= ~(1 << mode);
	
		# write value to ENABLE register
		self._write_byte_data(APDS9960_REG_ENABLE, reg_val)


	# start the light (R/G/B/Ambient) sensor
	def enableLightSensor(self, interrupts=True):
		self.setAmbientLightGain(APDS9960_DEFAULT_AGAIN)
		self.setAmbientLightIntEnable(interrupts)
		self.enablePower()
		self.setMode(APDS9960_MODE_AMBIENT_LIGHT, True)

	# stop the light sensor
	def disableLightSensor(self):
		self.setAmbientLightIntEnable(False)
		self.setMode(APDS9960_MODE_AMBIENT_LIGHT, False)


	# start the proximity sensor
	def enableProximitySensor(self, interrupts=True):
		self.setProximityGain(APDS9960_DEFAULT_PGAIN)
		self.setLEDDrive(APDS9960_DEFAULT_LDRIVE)
		self.setProximityIntEnable(interrupts)
		self.enablePower()
		self.setMode(APDS9960_MODE_PROXIMITY, True)

	# stop the proximity sensor
	def disableProximitySensor(self):
		self.setProximityIntEnable(False)
		self.setMode(APDS9960_MODE_PROXIMITY, False)


	# start the gesture recognition engine
	def enableGestureSensor(self, interrupts=True):
		self.resetGestureParameters()
		self._write_byte_data(APDS9960_REG_WTIME, 0xff)
		self._write_byte_data(APDS9960_REG_PPULSE, APDS9960_DEFAULT_GESTURE_PPULSE)
		self.setLEDBoost(APDS9960_LED_BOOST_300)
		self.setGestureIntEnable(interrupts)
		self.setGestureMode(True)
		self.enablePower()
		self.setMode(APDS9960_MODE_WAIT, True)
		self.setMode(APDS9960_MODE_PROXIMITY, True)
		self.setMode(APDS9960_MODE_GESTURE, True)

	# stop the gesture recognition engine
	def disableGestureSensor(self):
		self.resetGestureParameters()
		self.setGestureIntEnable(False)
		self.setGestureMode(False)
		self.setMode(APDS9960_MODE_GESTURE, False)


	# check if there is a gesture available
	def isGestureAvailable(self):
		val = self._read_byte_data(APDS9960_REG_GSTATUS)
	
		# shift and mask out GVALID bit
		val &= APDS9960_BIT_GVALID;
	
		return (val == APDS9960_BIT_GVALID)


	# processes a gesture event and returns best guessed gesture
	def readGesture(self):
		fifo_level = 0
		bytes_read = 0
		fifo_data = []
	
		# make sure that power and gesture is on and data is valid
		if not (self.getMode() & 0b01000001) or not self.isGestureAvailable():
			return APDS9960_DIR_NONE
	
		# keep looping as long as gesture data is valid
		while(self.isGestureAvailable()):
			# read the current FIFO level
			fifo_level = self._read_byte_data(APDS9960_REG_GFLVL)

			# if there's stuff in the FIFO, read it into our data block
			if fifo_level > 0:
				fifo_data = []
				for i in range(0, fifo_level):
					fifo_data += self._read_i2c_block_data(APDS9960_REG_GFIFO_U, 4)

				# if at least 1 set of data, sort the data into U/D/L/R
				if len(fifo_data) >= 4:
					for i in range(0, len(fifo_data), 4):
						self.gesture_data_.u_data[self.gesture_data_.index] = fifo_data[i + 0]
						self.gesture_data_.d_data[self.gesture_data_.index] = fifo_data[i + 1]
						self.gesture_data_.l_data[self.gesture_data_.index] = fifo_data[i + 2]
						self.gesture_data_.r_data[self.gesture_data_.index] = fifo_data[i + 3]
						self.gesture_data_.index += 1
						self.gesture_data_.total_gestures += 1

					# filter and process gesture data, decode near/far state
					if self.processGestureData():
						if self.decodeGesture():
							#***TODO: U-Turn Gestures
							pass
					# reset data
					self.gesture_data_.index = 0
					self.gesture_data_.total_gestures = 0
			# wait some time to collect next batch of FIFO data
			sleep(APDS9960_TIME_FIFO_PAUSE)
		# determine best guessed gesture and clean up
		sleep(APDS9960_TIME_FIFO_PAUSE)
		self.decodeGesture()
		motion = self.gesture_motion_
		self.resetGestureParameters()
		if motion == 1 :
			return "LEFT" , motion 
		elif motion == 2 :
			return "RIGHT" , motion 
		elif motion == 3 :
			return "UP" , motion 
		elif motion == 4 :
			return "DOWN" , motion 
		else :
			return "NONE" , motion
		
	# turn the APDS-9960 on
	def enablePower(self):
		self.setMode(APDS9960_MODE_POWER, True)
	def disablePower(self):
		self.setMode(APDS9960_MODE_POWER, False)
	# ambient light and color sensor controls
	def readAmbientLight(self):
		l = self._read_byte_data(APDS9960_REG_CDATAL)
		h = self._read_byte_data(APDS9960_REG_CDATAH)
		return l + (h << 8)
	def readRedLight(self):
		l = self._read_byte_data(APDS9960_REG_RDATAL)
		h = self._read_byte_data(APDS9960_REG_RDATAH)
		return l + (h << 8)
	def readGreenLight(self):
		l = self._read_byte_data(APDS9960_REG_GDATAL)
		h = self._read_byte_data(APDS9960_REG_GDATAH)
		return l + (h << 8)
	def readBlueLight(self):
		l = self._read_byte_data(APDS9960_REG_BDATAL)
		h = self._read_byte_data(APDS9960_REG_BDATAH)
		return l + (h << 8)
	# Proximity sensor controls
	def readProximity(self):
		return self._read_byte_data(APDS9960_REG_PDATA)
	# High-level gesture controls
	def resetGestureParameters(self):
		self.gesture_data_.index = 0
		self.gesture_data_.total_gestures = 0
		self.gesture_ud_delta_ = 0
		self.gesture_lr_delta_ = 0
		self.gesture_ud_count_ = 0
		self.gesture_lr_count_ = 0
		self.gesture_near_count_ = 0
		self.gesture_far_count_ = 0
		self.gesture_state_ = 0
		self.gesture_motion_ = APDS9960_DIR_NONE
	def processGestureData(self):
		u_first = 0
		d_first = 0
		l_first = 0
		r_first = 0
		u_last = 0
		d_last = 0
		l_last = 0
		r_last = 0
		# if we have less than 4 total gestures, that's not enough
		if self.gesture_data_.total_gestures <= 4:
			return False
		# check to make sure our data isn't out of bounds
		if self.gesture_data_.total_gestures <= 32 and self.gesture_data_.total_gestures > 0:
			# find the first value in U/D/L/R above the threshold
			for i in range(0, self.gesture_data_.total_gestures):
				if self.gesture_data_.u_data[i] > APDS9960_GESTURE_THRESHOLD_OUT and \
					self.gesture_data_.d_data[i] > APDS9960_GESTURE_THRESHOLD_OUT and \
					self.gesture_data_.l_data[i] > APDS9960_GESTURE_THRESHOLD_OUT and \
					self.gesture_data_.r_data[i] > APDS9960_GESTURE_THRESHOLD_OUT:
					u_first = self.gesture_data_.u_data[i]
					d_first = self.gesture_data_.d_data[i]
					l_first = self.gesture_data_.l_data[i]
					r_first = self.gesture_data_.r_data[i]
					break
			# if one of the _first values is 0, then there is no good data
			if u_first == 0 or	d_first == 0 or l_first == 0 or r_first == 0:
				return False
			# find the last value in U/D/L/R above the threshold
			for i in reversed(range(0, self.gesture_data_.total_gestures)):
				if self.gesture_data_.u_data[i] > APDS9960_GESTURE_THRESHOLD_OUT and \
					self.gesture_data_.d_data[i] > APDS9960_GESTURE_THRESHOLD_OUT and \
					self.gesture_data_.l_data[i] > APDS9960_GESTURE_THRESHOLD_OUT and \
					self.gesture_data_.r_data[i] > APDS9960_GESTURE_THRESHOLD_OUT:
					u_last = self.gesture_data_.u_data[i]
					d_last = self.gesture_data_.d_data[i]
					l_last = self.gesture_data_.l_data[i]
					r_last = self.gesture_data_.r_data[i]
					break
			# calculate the first vs. last ratio of up/down and left/right
			ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first)
			lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first)
			ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last)
			lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last)
			# determine the difference between the first and last ratios
			ud_delta = ud_ratio_last - ud_ratio_first
			lr_delta = lr_ratio_last - lr_ratio_first
			# accumulate the UD and LR delta values
			self.gesture_ud_delta_ += ud_delta
			self.gesture_lr_delta_ += lr_delta
			# determine U/D gesture
			if self.gesture_ud_delta_ >= APDS9960_GESTURE_SENSITIVITY_1:
				self.gesture_ud_count_ = 1
			elif self.gesture_ud_delta_ <= -APDS9960_GESTURE_SENSITIVITY_1:
				self.gesture_ud_count_ = -1
			else:
				self.gesture_ud_count_ = 0
			# determine L/R gesture
			if self.gesture_lr_delta_ >= APDS9960_GESTURE_SENSITIVITY_1:
				self.gesture_lr_count_ = 1
			elif self.gesture_lr_delta_ <= -APDS9960_GESTURE_SENSITIVITY_1:
				self.gesture_lr_count_ = -1
			else:
				self.gesture_lr_count_ = 0
			# determine Near/Far gesture
			if self.gesture_ud_count_ == 0 and self.gesture_lr_count_ == 0:
				if abs(ud_delta) < APDS9960_GESTURE_SENSITIVITY_2 and \
					abs(lr_delta) < APDS9960_GESTURE_SENSITIVITY_2:
					if ud_delta == 0 and lr_delta == 0:
						self.gesture_near_count_ += 1
					elif ud_delta != 0 or lr_delta != 0:
						self.gesture_far_count_ += 1
					if self.gesture_near_count_ >= 10 and self.gesture_far_count_ >= 2:
						if ud_delta == 0 and lr_delta == 0:
							self.gesture_state_ = APDS9960_STATE_NEAR
						elif ud_delta != 0 and lr_delta != 0:
							self.gesture_state_ = APDS9960_STATE_FAR
						return True
			else:
				if abs(ud_delta) < APDS9960_GESTURE_SENSITIVITY_2 and \
					abs(lr_delta) < APDS9960_GESTURE_SENSITIVITY_2:
						if ud_delta == 0 and lr_delta == 0:
							self.gesture_near_count_ += 1
						if self.gesture_near_count_ >= 10:
							self.gesture_ud_count_ = 0
							self.gesture_lr_count_ = 0
							self.gesture_ud_delta_ = 0
							self.gesture_lr_delta_ = 0

		return False

	def decodeGesture(self):
		# return if near or far event is detected
		if self.gesture_state_ == APDS9960_STATE_NEAR:
			self.gesture_motion_ = APDS9960_DIR_NEAR
			return True
		if self.gesture_state_ == APDS9960_STATE_FAR:
			self.gesture_motion_ = APDS9960_DIR_FAR
			return True
		# determine swipe direction
		if self.gesture_ud_count_ == -1 and self.gesture_lr_count_ == 0:
			self.gesture_motion_ = APDS9960_DIR_UP
		elif self.gesture_ud_count_ == 1 and self.gesture_lr_count_ == 0:
			self.gesture_motion_ = APDS9960_DIR_DOWN
		elif self.gesture_ud_count_ == 0 and self.gesture_lr_count_ == 1:
			self.gesture_motion_ = APDS9960_DIR_RIGHT
		elif self.gesture_ud_count_ == 0 and self.gesture_lr_count_ == -1:
			self.gesture_motion_ = APDS9960_DIR_LEFT
		elif self.gesture_ud_count_ == -1 and self.gesture_lr_count_ == 1:
			if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
				self.gesture_motion_ = APDS9960_DIR_UP
			else:
				self.gesture_motion_ = APDS9960_DIR_DOWN
		elif self.gesture_ud_count_ == 1 and self.gesture_lr_count_ == -1:
			if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
				self.gesture_motion_ = APDS9960_DIR_DOWN
			else:
				self.gesture_motion_ = APDS9960_DIR_LEFT
		elif self.gesture_ud_count_ == -1 and self.gesture_lr_count_ == -1:
			if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
				self.gesture_motion_ = APDS9960_DIR_UP
			else:
				self.gesture_motion_ = APDS9960_DIR_LEFT
		elif self.gesture_ud_count_ == 1 and self.gesture_lr_count_ == 1:
			if abs(self.gesture_ud_delta_) > abs(self.gesture_lr_delta_):
				self.gesture_motion_ = APDS9960_DIR_DOWN
			else:
				self.gesture_motion_ = APDS9960_DIR_RIGHT
		else:
			return False
		return True
	# Getters and setters for register values
	def getProxIntLowThresh(self):
		return self._read_byte_data(APDS9960_REG_PILT)
	def setProxIntLowThresh(self, threshold):
		self._write_byte_data(APDS9960_REG_PILT, threshold)
	def getProxIntHighThresh(self):
		return self._read_byte_data(APDS9960_REG_PIHT)
	def setProxIntHighThresh(self, threshold):
		self._write_byte_data(APDS9960_REG_PIHT, threshold)
	def getLEDDrive(self):
		val = self._read_byte_data(APDS9960_REG_CONTROL)		
		return (val >> 6) & 0b00000011
	def setLEDDrive(self, drive):
		val = self._read_byte_data(APDS9960_REG_CONTROL)		
		drive &= 0b00000011
		drive = drive << 6
		val &= 0b00111111
		val |= drive
		self._write_byte_data(APDS9960_REG_CONTROL, val)
	def getProximityGain(self):
		val = self._read_byte_data(APDS9960_REG_CONTROL)		
		return (val >> 2) & 0b00000011
	def setProximityGain(self, drive):
		val = self._read_byte_data(APDS9960_REG_CONTROL)		
		drive &= 0b00000011
		drive = drive << 2
		val &= 0b11110011
		val |= drive
		self._write_byte_data(APDS9960_REG_CONTROL, val)
	def getAmbientLightGain(self):
		val = self._read_byte_data(APDS9960_REG_CONTROL)		
		return (val & 0b00000011)
	def setAmbientLightGain(self, drive):
		val = self._read_byte_data(APDS9960_REG_CONTROL)		
		drive &= 0b00000011
		val &= 0b11111100
		val |= drive
		self._write_byte_data(APDS9960_REG_CONTROL, val)
	def getLEDBoost(self):
		val = self._read_byte_data(APDS9960_REG_CONFIG2)
		return (val >> 4) & 0b00000011
	def setLEDBoost(self, boost):
		val = self._read_byte_data(APDS9960_REG_CONFIG2)
		boost &= 0b00000011
		boost = boost << 4
		val &= 0b11001111
		val |= boost
		self._write_byte_data(APDS9960_REG_CONFIG2, val)
	def getProxGainCompEnable(self):
		val = self._read_byte_data(APDS9960_REG_CONFIG3)
		val = (val >> 5) & 0b00000001
		return val == 1
	def setProxGainCompEnable(self, enable):
		val = self._read_byte_data(APDS9960_REG_CONFIG3)
		val &= 0b11011111
		if enable:
			val |= 0b00100000
		self._write_byte_data(APDS9960_REG_CONFIG3, val)
	def getProxPhotoMask(self):
		val = self._read_byte_data(APDS9960_REG_CONFIG3)
		return val & 0b00001111
	def setProxPhotoMask(self, mask):
		val = self._read_byte_data(APDS9960_REG_CONFIG3)
		mask &= 0b00001111
		val &= 0b11110000
		val |= mask
		self._write_byte_data(APDS9960_REG_CONFIG3, val)
	def getGestureEnterThresh(self):
		return self._read_byte_data(APDS9960_REG_GPENTH)
	def setGestureEnterThresh(self, threshold):
		self._write_byte_data(APDS9960_REG_GPENTH, threshold)
	def getGestureExitThresh(self):
		return self._read_byte_data(APDS9960_REG_GEXTH)
	def setGestureExitThresh(self, threshold):
		self._write_byte_data(APDS9960_REG_GEXTH, threshold)
	def getGestureGain(self):
		val = self._read_byte_data(APDS9960_REG_GCONF2)		   
		return (val >> 5) & 0b00000011
	def setGestureGain(self, gain):
		val = self._read_byte_data(APDS9960_REG_GCONF2)		   
		gain &= 0b00000011
		gain = gain << 5
		val &= 0b10011111
		val |= gain
		self._write_byte_data(APDS9960_REG_GCONF2, val)
	def getGestureLEDDrive(self):
		val = self._read_byte_data(APDS9960_REG_GCONF2)		   
		return (val >> 3) & 0b00000011
	def setGestureLEDDrive(self, drive):
		val = self._read_byte_data(APDS9960_REG_GCONF2)		   
		drive &= 0b00000011;
		drive = drive << 3;
		val &= 0b11100111;
		val |= drive;
		self._write_byte_data(APDS9960_REG_GCONF2, val)
	def getGestureWaitTime(self):
		val = self._read_byte_data(APDS9960_REG_GCONF2)		   
		return val & 0b00000111
	def setGestureWaitTime(self, time):
		val = self._read_byte_data(APDS9960_REG_GCONF2)		   
		time &= 0b00000111
		val &= 0b11111000
		val |= time
		self._write_byte_data(APDS9960_REG_GCONF2, val)
	def getLightIntLowThreshold(self):
		return self._read_byte_data(APDS9960_REG_AILTL) | (self._read_byte_data(APDS9960_REG_AILTH) << 8)
	def setLightIntLowThreshold(self, threshold):
		self._write_byte_data(APDS9960_REG_AILTL, threshold & 0x00ff)
		self._write_byte_data(APDS9960_REG_AILTH, (threshold & 0xff00) >> 8)
	def getLightIntHighThreshold(self):
		return self._read_byte_data(APDS9960_REG_AIHTL) | (self._read_byte_data(APDS9960_REG_AIHTH) << 8)
	def setLightIntHighThreshold(self, threshold):
		self._write_byte_data(APDS9960_REG_AIHTL, threshold & 0x00ff)
		self._write_byte_data(APDS9960_REG_AIHTH, (threshold & 0xff00) >> 8)
	def getProximityIntLowThreshold(self):
		return self._read_byte_data(APDS9960_REG_PILT)
	def setProximityIntLowThreshold(self, threshold):
		self._write_byte_data(APDS9960_REG_PILT, threshold)
	def getProximityIntHighThreshold(self):
		return self._read_byte_data(APDS9960_REG_PIHT)
	def setProximityIntHighThreshold(self, threshold):
		self._write_byte_data(APDS9960_REG_PIHT, threshold)
	def getAmbientLightIntEnable(self):
		val = self._read_byte_data(APDS9960_REG_ENABLE)
		return (val >> 4) & 0b00000001 == 1
	def setAmbientLightIntEnable(self, enable):
		val = self._read_byte_data(APDS9960_REG_ENABLE)
		val &= 0b11101111;
		if enable:
			val |= 0b00010000
		self._write_byte_data(APDS9960_REG_ENABLE, val)
	def getProximityIntEnable(self):
		val = self._read_byte_data(APDS9960_REG_ENABLE)
		return (val >> 5) & 0b00000001 == 1
	def setProximityIntEnable(self, enable):
		val = self._read_byte_data(APDS9960_REG_ENABLE)
		val &= 0b11011111;
		if enable:
			val |= 0b00100000
		self._write_byte_data(APDS9960_REG_ENABLE, val)
	def getGestureIntEnable(self):
		val = self._read_byte_data(APDS9960_REG_GCONF4)		   
		return (val >> 1) & 0b00000001 == 1
	def setGestureIntEnable(self, enable):
		val = self._read_byte_data(APDS9960_REG_GCONF4)
		val &= 0b11111101
		if enable:
			val |= 0b00000010
		self._write_byte_data(APDS9960_REG_GCONF4, val)
	def clearAmbientLightInt(self):
		self._read_byte_data(APDS9960_REG_AICLEAR)	  
	def clearProximityInt(self):
		self._read_byte_data(APDS9960_REG_PICLEAR)	  
	def getGestureMode(self):
		val = self._read_byte_data(APDS9960_REG_GCONF4)		   
		return val & 0b00000001 == 1
	def setGestureMode(self, enable):
		val = self._read_byte_data(APDS9960_REG_GCONF4)
		val &= 0b11111110
		if enable:
			val |= 0b00000001
		self._write_byte_data(APDS9960_REG_GCONF4, val)	 
	def _read_byte_data(self, cmd):
		return self.bus.readfrom_mem(self.address, cmd,1)[0]
	def _write_byte_data(self, cmd, val):
		temp = bytearray(1)
		temp[0] = val 
		return self.bus.writeto_mem(self.address, cmd, temp)
	def _read_i2c_block_data(self, cmd, num):
		return self.bus.readfrom_mem(self.address, cmd, num)