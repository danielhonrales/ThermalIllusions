import RPi.GPIO as GPIO
import serial
import time

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=9600,
    timeout=1
)

thermal_pins = [40, 38, 36, 32]

for voltPin in thermal_pins:
    GPIO.setup(voltPin, GPIO.OUT)
    GPIO.output(voltPin, GPIO.LOW)

polarity_pin_1 = thermal_pins[0]
polarity_pin_2 = thermal_pins[3]
GPIO.output(polarity_pin_1, GPIO.HIGH)
GPIO.output(polarity_pin_2, GPIO.HIGH)

ser.write(f'VSET1:{0.5}'.encode())
time.sleep(4)
ser.write(f'VSET1:{0}'.encode())

GPIO.output(polarity_pin_1, GPIO.LOW)
GPIO.output(polarity_pin_2, GPIO.LOW)