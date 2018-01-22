import math ,framebuf 
from SSD1306 import SSD1306, SSD1306_I2C
from machine import I2C,Pin
from blocky_hardware import *

class OLED :
	def __init__ ( self , PortName , width = 128 , height = 64 ):
		self.oled = SSD1306_I2C(width,height,I2C(scl=Pin(PortName["pin1"],Pin.PULL_UP),sda=Pin(PortName["pin2"],Pin.PULL_UP,freq = 400000),external_vcc = False)
		self.oled.init_display()
		
	class NUMBER :
		def __init__ ( self,x , y , digits):
			self.oled.fill_rect(x,y,x+digits*3+ 4 ,y+8+4,0)
			self.oled.rect(x,y,x+digits*3+ 4 ,y+8+4,0)
			self.lastNumber = None
			self.x = x
			self.y = y
			self.digits = digits
		def number (self, number):
			self.oled.fill_rect(x+2,y+2,x+2+self.digits*8,y+10,0)
			x_cord = self.x
			y_cord = self.y
			if number // 100 > digits :
				return 
			elif number // 100 == digits:
				self.oled.text(str(number) , x+2 , y+2 , 1)
			else:
				self.oled.text(str(number) , x+2 + (digits-number//100) , y+2 , 1)

				
oled = OLED.NUMBER(Port3)
