import RPi.GPIO as GPIO
import time
servopin = 20
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(servopin, GPIO.OUT, initial=False)
p = GPIO.PWM(servopin,50) 
p.start(12.5)
time.sleep(5)
p.ChangeDutyCycle(2.5)
time.sleep(1)
while(True):
	time.sleep(1)
	print "anything?"
	p.ChangeDutyCycle(10)
	time.sleep(1)
	p.ChangeDutyCycle(3)
