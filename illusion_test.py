import RPi.GPIO as GPIO
from time import sleep, time

tactile_pins = [
        37,
        35,
        33,
        31,
        29,
        23,
        21,
        19,
        15
    ]

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# PWM Setup
pwm = []
cycle = 20  # ms
for motor in tactile_pins[:3]:
    print(f"Setting up motor {motor}")
    GPIO.setup(motor, GPIO.OUT)
    GPIO.output(motor, GPIO.LOW)

    pwmMotor = GPIO.PWM(motor, 1 / cycle * 100000)
    pwm.append(pwmMotor)

pwm[0].start(0)    
pwm[1].start(0)
pwm[2].start(0)

def pulse(pwmIndex, duration, intensity):
    pwm[pwmIndex].ChangeDutyCycle(intensity)
    sleep(duration)
    pwm[pwmIndex].ChangeDutyCycle(0)

pulse_duration = .024
pulse_interval = .03
ibi = .024

while True:
    pulse(0, pulse_duration, 100)
    sleep(pulse_interval)
    pulse(0, pulse_duration, 100)
    sleep(pulse_interval)
    pulse(0, pulse_duration, 100)

    sleep(ibi)
    pulse(0, pulse_duration, 100)
    sleep(pulse_interval)
    pulse(0, pulse_duration, 100)
    sleep(pulse_interval)
    pulse(0, pulse_duration, 100)
    
    sleep(ibi)
    pulse(2, pulse_duration, 100)
    sleep(pulse_interval)
    pulse(2, pulse_duration, 100)
    sleep(pulse_interval)
    pulse(2, pulse_duration, 100)

    sleep(3)
    """ pwm[1].ChangeDutyCycle(100)
    sleep(.15)
    pwm[1].ChangeDutyCycle(0)

    sleep(.8)

    pwm[1].ChangeDutyCycle(100)
    sleep(.15)
    pwm[1].ChangeDutyCycle(0)
    sleep(.08)

    pwm[2].ChangeDutyCycle(100)
    sleep(.15)
    pwm[2].ChangeDutyCycle(0)

    pwm[1].ChangeDutyCycle(0)
    pwm[2].ChangeDutyCycle(0)
    sleep(3) """

""" while True:
    pwm[1].ChangeDutyCycle(75)
    pwm[2].ChangeDutyCycle(75)
    sleep(.2)

``
    pwm[1].ChangeDutyCycle(0)
    pwm[2].ChangeDutyCycle(0)
    sleep(3) """