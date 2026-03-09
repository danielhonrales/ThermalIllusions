import RPi.GPIO as GPIO
from time import sleep, time
import sys
import socket
import serial
import math
import random
import json
import asyncio

### Connection #############################################
ip = '172.16.136.237'
port = '25567'
############################################################

### Tactile ################################################

tactile_pins = [26, 24, 16]

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# PWM Setup
pwm = []
cycle = 20  # ms
for motor in tactile_pins[:9]:
    print(f"Setting up motor {motor}")
    GPIO.setup(motor, GPIO.OUT)
    GPIO.output(motor, GPIO.LOW)

    pwmMotor = GPIO.PWM(motor, 1 / cycle * 100000)
    pwmMotor.start(0)
    pwm.append(pwmMotor)

############################################################

### Thermal ################################################

thermal_pins = [40, 38, 36, 32]

for voltPin in thermal_pins:
    GPIO.setup(voltPin, GPIO.OUT)
    GPIO.output(voltPin, GPIO.LOW)

ser = serial.Serial(
    '/dev/ttyACM0'
)
ser.write(f'VSET1:{0}'.encode())
preheat_time = 1.25
warmup_time = 2

############################################################

### Keys ################################################

INDEX = "index"
WAIT_TIME = "waitTime"
DURATION = "duration"
INTENSITY = "intensity"
RAMP_UP = "rampUp"
RAMP_DOWN = "rampDown"
VOLTAGE = "voltage"
POLARITY = "polarity"

############################################################

async def main():
    # Set up connection
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((ip, int(port)))
    server_socket.listen(1)

    while True:
        print(f"Waiting for connection at {ip}:{port} ...")
        connection, client_address = server_socket.accept()
        print("Connection from ", client_address)

        await receive_message(connection)

    server_socket.close()
    GPIO.cleanup()
    deactivate_thermal()
    ser.close()

async def receive_message(connection):
    print("Ready to go! Send in a command...")

    try:
        # Receive the data
        decoded = ""
        while True:
            # Read data
            data = connection.recv(4096)
            if not data:
                break
            decoded += data.decode()
            if decoded.endswith("$"):
                break
        print(f"Full message: {decoded}")
        decoded = decoded[:-1]

        # Load data
        message_json = json.loads(decoded)
        tactile_pulses = message_json["tactilePulses"]
        thermal_pulses = message_json["thermalPulses"]
        
        await play_pattern(tactile_pulses, thermal_pulses)

        await receive_message(connection)

    except RuntimeError as e:
        print(f"Error: {e}")
    except json.decoder.JSONDecodeError as e:
        print(f"Error: {e}")

#############################################################################

async def play_pattern(tactile_pulses, thermal_pulses):
    async with asyncio.TaskGroup() as tg:
        for pulse in tactile_pulses:
            tg.create_task(pulse_async(
                pulse[INDEX],
                pulse[DURATION],
                pulse[INTENSITY],
                pulse[WAIT_TIME],
                pulse[RAMP_UP],
                pulse[RAMP_DOWN],
                ))
        
        for pulse in thermal_pulses:
            tg.create_task(activate_thermal_async(
                pulse[POLARITY],
                pulse[VOLTAGE],
                pulse[DURATION],
                pulse[WAIT_TIME],
            ))

#############################################################################

async def activate_thermal_async(temp_polarity, voltage, duration, waitTime):
    await asyncio.sleep(waitTime)
    if temp_polarity == 1:
        ser.write(f'VSET1:{voltage}'.encode())
        GPIO.output(thermal_pins[1], GPIO.HIGH)
        GPIO.output(thermal_pins[2], GPIO.HIGH)
    elif temp_polarity == -1:
        ser.write(f'VSET1:{voltage}'.encode())
        GPIO.output(thermal_pins[0], GPIO.HIGH)
        GPIO.output(thermal_pins[3], GPIO.HIGH)
    else:
        await deactivate_thermal_async(0)
    
    await asyncio.sleep(duration)
    await deactivate_thermal_async(0)

async def deactivate_thermal_async(waitTime):
    await asyncio.sleep(waitTime)
    ser.write('VSET1:0'.encode())
    for thermal_pin in thermal_pins:
        GPIO.output(thermal_pin, GPIO.LOW)

async def pulse_async(pwmIndex, duration, intensity, waitTime, rampUp = 0, rampDown = 0):
    await asyncio.sleep(waitTime)
    
    rampWindow = duration / 3
    rampStep = intensity / 3
        
    if rampUp == 1:
        pwm[pwmIndex].ChangeDutyCycle(rampStep)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 2)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 3)
        await asyncio.sleep(rampWindow / 3)
    else:
        pwm[pwmIndex].ChangeDutyCycle(intensity)
        await asyncio.sleep(rampWindow)
    
    await asyncio.sleep(rampWindow)

    if rampDown == 1:
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 3)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 2)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 1)
        await asyncio.sleep(rampWindow / 3)
    else:
        await asyncio.sleep(rampWindow)

    pwm[pwmIndex].ChangeDutyCycle(0)

#############################################################################

asyncio.run(main())