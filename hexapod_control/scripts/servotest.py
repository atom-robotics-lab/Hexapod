from adafruit_servokit import ServoKit
from time import sleep
kit = ServoKit(channels=16)
servo = 14

while True:
	a = input("enter:-")
	kit.servo[2].angle = int(a)
	
