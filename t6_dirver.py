import time
from time import sleep
import RPi.GPIO as GPIO


class Motor_Driver:
	CW = 1     # Clockwise Rotation
	CCW = 0    # Counterclockwise Rotation
	
	SPR = 200   # Steps per Revolution (360 / 1.8)
	delay_const = .00243
	
	RESOLUTION = {'Full': (0, 0, 0),
					  'Half': (1, 0, 0),
					  '1/4': (0, 1, 0),
					  '1/8': (1, 1, 0),
					  '1/16': (0, 0, 1),
					  '1/32': (1, 0, 1)}
					  
	def __init__(self, dir1, step1, mode1, dir2, step2, mode2):
		self.delay = self.delay_const
		self.DIR1 = dir1
		self.STEP1 = step1
		self.DIR2 = dir2
		self.STEP2 = step2

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.DIR1, GPIO.OUT)
		GPIO.setup(self.STEP1, GPIO.OUT)
		

		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.DIR2, GPIO.OUT)
		GPIO.setup(self.STEP2, GPIO.OUT)
		
		self.MODE1 = mode1 #(14, 15, 18)   # Microstep Resolution GPIO Pins
		GPIO.setup(self.MODE1, GPIO.OUT)

		self.MODE2 = mode2 #(14, 15, 18)   # Microstep Resolution GPIO Pins
		GPIO.setup(self.MODE2, GPIO.OUT)
		
		GPIO.output(self.MODE1, self.RESOLUTION['Full'])
		GPIO.output(self.MODE2, self.RESOLUTION['Full'])
		
	def run(self, rpm):#1 r /s
		start = time.time()

		if rpm > 0:
			GPIO.output(self.DIR1, self.CW)
			GPIO.output(self.DIR2, self.CW)
		else:
			GPIO.output(self.DIR1, self.CCW)
			GPIO.output(self.DIR2, self.CCW)
		
		rpm = abs(rpm)
		print(rpm)
		if rpm >= 60:
			if rpm < 240:	#240 max
				self.delay = self.delay_const/(rpm/60)
			else:
				self.delay = self.delay_const/4
		else:			
			self.delay = self.delay_const*(60/rpm)

		for x in range(self.SPR):
			GPIO.output(self.STEP1, GPIO.HIGH)
			GPIO.output(self.STEP2, GPIO.HIGH)
			
			sleep(self.delay)

			GPIO.output(self.STEP1, GPIO.LOW)
			GPIO.output(self.STEP2, GPIO.LOW)

			sleep(self.delay)

		end = time.time() 
		print(end-start)
		
def main():
    driver = Motor_Driver(20,21,(14, 15, 18), 26,19,(6, 5, 13))
	
    try:
		driver.run(30)
		driver.run(-30)
		# driver.run(120)
		# driver.run(180)
		# driver.run(240)
		# driver.run(-60)
		# driver.run(-120)
		# driver.run(-180)
		# driver.run(-240)
		
    except KeyboardInterrupt:
        print ("\nCtrl-C pressed.  Stopping PIGPIO and exiting...")
    finally:		
		GPIO.cleanup()

if __name__ == '__main__':
    main()		
		

		
		
		
		

		
