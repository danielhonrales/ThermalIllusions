import RPi.GPIO as GPIO
from time import sleep, time
import sys
import socket
import serial
import math
import random
import json

### Connection #############################################
ip = '172.16.136.142'
port = '25567'
############################################################

### Tactile ################################################

tactile_pins = [37,35,33,31,29,23,21,19,15]

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

global actuator1, actuator2
actuator1 = 7
actuator2 = 8
global voltage1, voltage2
voltage1 = 2.3
voltage2 = 3.5

tactile_ser = serial.Serial(
    '/dev/ttyACM1'
)
tactile_ser.write(f'VSET1:{voltage1}'.encode())

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
preheat_time = .3
warmup_time = 2
post_warmup_interval = 1

############################################################

def main():
    # Set up connection
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((ip, int(port)))
    server_socket.listen(1)

    while True:
        print(f"Waiting for connection at {ip}:{port} ...")
        connection, client_address = server_socket.accept()
        #connection.setblocking(False)
        print("Connection from ", client_address)

        receive_message(connection)

    server_socket.close()
    GPIO.cleanup()
    deactivate_thermal()
    ser.close()

def receive_message(connection):
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
        messageJson = json.loads(decoded)
        illusion = messageJson.get('illusion')        
        thermal_voltage = messageJson.get('thermalVoltage')
        if thermal_voltage is not None: thermal_voltage = float(thermal_voltage)
        pulse_duration = messageJson.get('duration')
        if pulse_duration is not None: pulse_duration = float(pulse_duration)
        location = messageJson.get('location')
        if location is not None: location = float(location)
        pulse_interval = messageJson.get('interval')
        direction = messageJson.get('direction')
        if direction is not None: direction = int(direction)


        if illusion == "funneling":
            print(f"Rendering Funneling+Masking with \n\tthermal_voltage: {thermal_voltage}V, \n\tpulse_duration: {pulse_duration}s, \n\tlocation: {location}\n")
            mask_funnel(thermal_voltage, pulse_duration, location)
        elif illusion == "saltation":
            print(f"Rendering Rabbit+Masking with \n\tthermal_voltage: {thermal_voltage}V, \n\tpulse_duration: {pulse_duration}s, \n\tdirection: {direction}")
            mask_rabbit(thermal_voltage, pulse_duration, direction)
        elif illusion == "motion":
            print(f"Rendering Mask+Motion with \n\tthermal_voltage: {thermal_voltage}V, \n\tpulse_duration: {pulse_duration}s, \n\tdirection: {direction}\n")
            mask_motion(thermal_voltage, pulse_duration, direction)

        deactivate_thermal()

        receive_message(connection)

    except RuntimeError as e:
        print(f"Error: {e}")
    except json.decoder.JSONDecodeError as e:
        print(f"Error: {e}")

#############################################################################

def mask_funnel(thermal_voltage, pulse_duration, location):
    preheat(thermal_voltage)    
    warmup(thermal_voltage, [actuator1, actuator2])

    funnel(pulse_duration, location)

    deactivate_thermal()

def funnel(duration, location):
    if (duration < .15):
        tactile_ser.write(f'VSET1:{voltage2}'.encode())

    location = float(location)
    intensity1 = 100 * math.sqrt(location / (location + (1 - location)))
    intensity2 = 100 * math.sqrt((1 - location) / (location + (1 - location)))

    if intensity1 != 0:
        intensity1 = max(60, intensity1)
    if intensity2 != 0:
        intensity2 = max(60, intensity2)  
    print(intensity1, intensity2)

    pwm[actuator1].ChangeDutyCycle(float(intensity1))
    pwm[actuator2].ChangeDutyCycle(float(intensity2))
    sleep(float(duration))
    pwm[actuator1].ChangeDutyCycle(0)
    pwm[actuator2].ChangeDutyCycle(0)

    if (duration < .15):
        tactile_ser.write(f'VSET1:{voltage1}'.encode())

#############################################################################

def mask_rabbit(thermal_voltage, pulse_duration, direction):
    global actuator1, actuator2
    if direction == 0:
        actuator1 = 7
        actuator2 = 8
    else:
        actuator1 = 8
        actuator2 = 7

    preheat(thermal_voltage)
    warmup(thermal_voltage, [actuator1])

    print(actuator1, actuator2)
    rabbit(pulse_duration, 3)

    deactivate_thermal()

def rabbit(pulse_duration, hops):
    if (pulse_duration < .15):
        tactile_ser.write(f'VSET1:{voltage2}'.encode())

    if hops == 3:
        hops = 3
        pulse(actuator1, pulse_duration, 100)
        sleep(.8)
        pulse(actuator1, pulse_duration, 100)
        sleep(.05)
        pulse(actuator2, pulse_duration, 100)

    else:
        hops = 9
        for _ in range(int(hops / 3 * 2)):
            pulse(actuator1, pulse_duration, 100)
            sleep(pulse_interval)

        for _ in range(int(hops / 3)):
            pulse(actuator2, pulse_duration, 100)
            sleep(pulse_interval)
        
    if (pulse_duration < .15):
        tactile_ser.write(f'VSET1:{voltage1}'.encode())

#############################################################################

def mask_motion(thermal_voltage, pulse_duration, direction):
    global actuator1, actuator2
    if direction == 0:
        actuator1 = 7
        actuator2 = 8
    else:
        actuator1 = 8
        actuator2 = 7
    
    preheat(thermal_voltage)
    warmup(thermal_voltage, [actuator1])

    interval = ((0.32 * pulse_duration * 1000) + 47.3) / 1000

    motion(pulse_duration, interval)

    deactivate_thermal()

def motion(pulse_duration, interval):
    if (pulse_duration < .15):
        tactile_ser.write(f'VSET1:{voltage2}'.encode())

    pwm[actuator1].ChangeDutyCycle(75)
    sleep(interval)
    pwm[actuator2].ChangeDutyCycle(75)
    sleep(pulse_duration - interval)
    pwm[actuator1].ChangeDutyCycle(0)
    sleep(interval)
    pwm[actuator2].ChangeDutyCycle(0)

    if (pulse_duration < .15):
        tactile_ser.write(f'VSET1:{voltage1}'.encode())

#############################################################################

def mask(pulse_duration):
    temp_polarity = 1

    activate_thermal(temp_polarity, voltage)
    pwm[actuator1].ChangeDutyCycle(100)
    pwm[actuator2].ChangeDutyCycle(100)
    sleep(5)
    pwm[actuator1].ChangeDutyCycle(0)
    pwm[actuator2].ChangeDutyCycle(0)
    deactivate_thermal()

def full_mask(pulse_duration):
    temp_polarity = 1

    activate_thermal(temp_polarity, voltage)
    pulse(actuator1, pulse_duration, 100)
    deactivate_thermal()

    sleep(3)

    activate_thermal(temp_polarity, voltage)
    pulse(actuator2, pulse_duration, 100)
    deactivate_thermal()

    sleep(3)

    activate_thermal(temp_polarity, voltage)
    pwm[actuator1].ChangeDutyCycle(100)
    pwm[actuator2].ChangeDutyCycle(100)
    sleep(5)
    pwm[actuator1].ChangeDutyCycle(0)
    pwm[actuator2].ChangeDutyCycle(0)
    deactivate_thermal()

#############################################################################

def get_thermal_params(thermal_voltage):
    temp_polarity = 0
    if thermal_voltage > 0: temp_polarity = -1
    if thermal_voltage < 0: temp_polarity = 1
    thermal_voltage = abs(thermal_voltage)
    return (temp_polarity, thermal_voltage)

def preheat(thermal_voltage):
    activate_thermal(*get_thermal_params(thermal_voltage))    
    sleep(preheat_time)

def warmup(thermal_voltage, actuators):
    activate_thermal(*get_thermal_params(thermal_voltage))

    for actuator in actuators:
        pwm[actuator].ChangeDutyCycle(100)
    sleep(warmup_time)
    for actuator in actuators:
        pwm[actuator].ChangeDutyCycle(0)

    deactivate_thermal()
    sleep(post_warmup_interval - preheat_time)
    activate_thermal(*get_thermal_params(thermal_voltage))
    sleep(preheat_time)

def activate_thermal(temp_polarity, voltage):
    polarity_pin_1 = thermal_pins[0] if temp_polarity == 1 else thermal_pins[1]
    polarity_pin_2 = thermal_pins[3] if temp_polarity == 1 else thermal_pins[2]
    ser.write(f'VSET1:{voltage}'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.HIGH)
        GPIO.output(polarity_pin_2, GPIO.HIGH)

def deactivate_thermal():
    ser.write('VSET1:0'.encode())
    for thermal_pin in thermal_pins:
        GPIO.output(thermal_pin, GPIO.LOW)

def thermal_pulse(pulse_duration, pulse_interval):
    activate_thermal(1, 1.3)

    pwm[actuator1].ChangeDutyCycle(100)
    sleep(3)
    pwm[actuator1].ChangeDutyCycle(0)
    sleep(.5)

    for _ in range(5):
        #activate_thermal(1, 1.3)
        pulse(actuator1, pulse_duration, 100)
        #deactivate_thermal()
        sleep(pulse_interval)

    deactivate_thermal()

def pulse(pwmIndex, duration, intensity):
    pwm[pwmIndex].ChangeDutyCycle(intensity)
    sleep(duration)
    pwm[pwmIndex].ChangeDutyCycle(0)

main()