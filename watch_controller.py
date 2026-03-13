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
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, int(port)))

    while True:
        print(f"Waiting for message from {ip}:{port} ...")
        
        while True:
            await receive_message(sock)

    sock.close()
    GPIO.cleanup()
    deactivate_thermal()
    ser.close()

current_pattern_task = None  # track the running pattern globally

async def receive_message(sock):
    global current_pattern_task
    print("Ready to go! Send in a command...")
    try:
        while True:
            data, addr = sock.recvfrom(4096)
            if not data:
                continue
            message = data.decode("ascii")
            if message.endswith("$"):
                message = message[:-1]

            #print(get_master_time())
            print(f"Full message: {message}")

            # ── Handle interrupt ───────────────────────────
            if message == "interrupt":
                if current_pattern_task and not current_pattern_task.done():
                    current_pattern_task.cancel()
                    await deactivate_thermal_async(0)
                    for p in pwm:
                        p.ChangeDutyCycle(0)
                    print("Pattern interrupted")
                continue  # wait for next message

            # ── Handle pattern ─────────────────────────────
            message_json = json.loads(message)
            send_time = message_json["sendTime"]
            latency = get_master_time() - send_time
            if message_json["device"] == "android":
                latency += 3300
            print(f"Latency: {latency}")

            tactile_pulses = message_json["tactilePulses"]
            thermal_pulses = message_json["thermalPulses"]
            start_delay = max(0, latency)

            # Cancel any currently running pattern
            if current_pattern_task and not current_pattern_task.done():
                current_pattern_task.cancel()
                await deactivate_thermal_async(0)
                for p in pwm:
                    p.ChangeDutyCycle(0)

            # Start new pattern as cancellable task
            current_pattern_task = asyncio.create_task(
                play_pattern(start_delay, tactile_pulses, thermal_pulses)
            )
            await asyncio.sleep(0)

    except RuntimeError as e:
        print(f"Error: {e}")
    except json.decoder.JSONDecodeError as e:
        print(f"Error: {e}")

#############################################################################
async def play_pattern(start_delay, tactile_pulses, thermal_pulses):
    try:
        await asyncio.sleep(start_delay / 1000)
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
        print("Done")
    except asyncio.CancelledError:
        print("Pattern cancelled")
        raise  # must re-raise so asyncio cleans up properly

#############################################################################
async def pulse_async(pwmIndex, duration, intensity, waitTime, rampUp=0, rampDown=0):
    try:
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

    except asyncio.CancelledError:
        pwm[pwmIndex].ChangeDutyCycle(0)  # clean up on cancel
        raise

async def activate_thermal_async(temp_polarity, voltage, duration, waitTime):
    try:
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

    except asyncio.CancelledError:
        await deactivate_thermal_async(0)  # clean up on cancel
        raise

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

def get_master_time():
    return int(time() * 1000)
#############################################################################

asyncio.run(main())
