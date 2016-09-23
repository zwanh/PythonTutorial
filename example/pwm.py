#this is an example of pwm

import wiringpi as gpio

gpio.wiringPiSetup()
gpio.pinMode(1, 2)
gpio.pwmSetClock(3840)
gpio.pwmSetRange(200000)
gpio.pwmWrite(1, 20000)
#gpio.pinMode(4, 2)
#gpio.pwmWrite(4, 512)
