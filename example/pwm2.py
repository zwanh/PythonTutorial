import RPi.GPIO as GPIO
import time
servopin = 20
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(servopin, GPIO.OUT, initial=False)
p = GPIO.PWM(servopin,50) 
p.start(12.5)
time.sleep(2)
p.ChangeDutyCycle(2.5)
time.sleep(2)
p.ChangeDutyCycle(7.5)
while(False):
	time.sleep(1)
	print "anything"
	p.ChangeDutyCycle(10)
	time.sleep(1)
	p.ChangeDutyCycle(3)
