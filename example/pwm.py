#this is an example of pwm

import wiringpi2 as gpio

PIN_TO_PWM = 1
OUT_PUT = 1

gpio.wiringPiSetup()
gpio.pinMode(PIN_TO_PWM, OUT_PUT)

gpio.softPwmCreate(PIN_TO_PWM, 0, 100)

for time in range(0, 4):
	for brightness in range(0, 100):
		gpio.softPwmWrite(PIN_TO_PWM, brightness)
		gpio.delay(10)
	for brightness in reversed(range(0, 100)):
		gpio.softPwmWrite(PIN_TO_PWM, brightness)
		gpio.delay(10)
