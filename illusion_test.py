import RPi.GPIO as GPIO
from time import sleep, time
import sys
import serial
import math
import random

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

actuator1 = 7
actuator2 = 8
body_location = "arm"

############################################################

### Thermal ################################################

thermal_pins = [40, 38, 36, 32]

for voltPin in thermal_pins:
    GPIO.setup(voltPin, GPIO.OUT)
    GPIO.output(voltPin, GPIO.LOW)

global ser
ser = serial.Serial(
    '/dev/ttyACM0'
)
ser.write(f'VSET1:{0}'.encode())
voltage = 1.5
preheat_time = .75
warmup_time = 2

############################################################

def main():
    prev_params = None
    prev_prev_params = None
    while True:
        command = input("Input command: ")
        print("Received: " + command + "\n")

        params = command.split(" ")
        loop = False

        if params[0] == ".":
            print("Using previous command")
            params = prev_params
        if params[0] == "..":
            print("Using previous previous command")
            params = prev_prev_params
        if params[0] in "loop":
            loop = True
            params = params[1:] if len(params) > 1 else prev_params

        save_params = handle_params(params, loop)

        if save_params: 
            prev_prev_params = prev_params
            prev_params = params

def handle_params(params, loop):
    save_params = True

    try:
        while True:
            if params[0] in "mask":
                thingy = 1
                polarity = 1

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 2:
                    polarity = int (params[1])
                    thingy = int(params[2])
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering Mask with \n\tpolarity: {polarity}\n, \n\thingy: {thingy}\n")        
                mask(polarity, thingy)
            elif params[0] in "maskrabbit" and params[0] not in "rabbit":
                pulse_duration = .048
                pulse_interval = .06
                hops = 9
                warmup = 1

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 1:
                    multiplier = float(params[1])
                    pulse_duration, pulse_interval = pulse_duration * multiplier, pulse_interval * multiplier
                elif len(params[1:]) > 1:
                    pulse_duration, pulse_interval, hops, warmup = params[1:]
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering Rabbit+Masking with \n\tpulse_duration: {pulse_duration}ms, \n\tpulse_interval: {pulse_interval}ms\n")
                mask_rabbit(float(pulse_duration), float(pulse_interval), float(hops), float(warmup) == 1)

            elif params[0] in "rabbit":
                pulse_duration = .048
                pulse_interval = .024
                hops = 9

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 1:
                    multiplier = float(params[1])
                    pulse_duration, pulse_interval = pulse_duration * multiplier, pulse_interval * multiplier
                elif len(params[1:]) == 2:
                    pulse_duration, pulse_interval = params[1:]
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering Rabbit with \n\tpulse_duration: {pulse_duration}ms, \n\tpulse_interval: {pulse_interval}ms\n")
                rabbit(float(pulse_duration), float(pulse_interval), hops)

            elif params[0] in "maskfunnel" and params[0] not in "funnel":
                pulse_duration = .5
                location = .5
                warmup = 1

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 1:
                    if params[1] == "rand":
                        location = random.random()
                    else:
                        location = float(params[1])
                elif len(params[1:]) > 1:
                    pulse_duration, location, warmup = params[1:]
                    if params[2] == "rand":
                        location = [0, 0.3, 0.5, 0.7, 1.0][random.randint(0, 4)]
                    
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering Test with \n\tpulse_duration: {pulse_duration}ms, \n\tlocation: {location}\n")        
                mask_funnel(float(pulse_duration), float(location), float(warmup))

            elif params[0] in "funnel":
                pulse_duration = .224
                location = 0.5

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 1:
                    location = float(params[1])
                elif len(params[1:]) == 2:
                    pulse_duration, location = params[1]
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering Funnel with \n\tpulse_duration: {pulse_duration}ms, \n\tlocation: {location}\n")        
                funnel(pulse_duration, location)

            elif params[0] in "maskmotion" and params[0] not in "motion":
                pulse_duration = .2
                interval = .13
                warmup = 1

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 1:
                    multiplier = float(params[1])
                    interval = interval * multiplier
                elif len(params[1:]) > 1:
                    pulse_duration, interval, warmup = params[1:]
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering MaskMotion with \n\tpulse_duration: {pulse_duration}ms, \n\tinterval: {interval}ms\n")        
                mask_motion(pulse_duration, interval, float(warmup) == 1)

            elif params[0] in "motion":
                pulse_duration = .2
                interval = .13

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 1:
                    multiplier = float(params[1])
                    interval = interval * multiplier
                elif len(params[1:]) == 2:
                    pulse_duration, interval = params[1:]
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering Motion with \n\tpulse_duration: {pulse_duration}ms, \n\tinterval: {interval}ms\n")        
                motion(pulse_duration, interval)

            elif params[0] in "fullmask":
                pulse_duration = 5

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 1:
                    pulse_duration = float(params[1])
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering Mask with \n\tpulse_duration: {pulse_duration}ms\n")        
                full_mask(pulse_duration)

            elif params[0] in "thermalpulse":
                pulse_duration = 1
                pulse_interval = 1

                if len(params[1:]) == 0:
                    print("Using default config")
                elif len(params[1:]) == 2:
                    pulse_duration, pulse_interval = params[1:]
                else:
                    print("Incorrect config \n")
                    break

                print(f"Rendering ThermalPulse\n")
                thermal_pulse(float(pulse_duration), float(pulse_interval))

            elif params[0] in "reverse":
                global actuator1, actuator2
                temp = actuator1
                actuator1 = actuator2
                actuator2 = temp
                print(f"Swapped actuator1 ({actuator1}) and actuator2 ({actuator2})")

            elif params[0] in "swapactuators":
                global body_location
                if body_location == "arm":
                    body_location = "finger"
                    if actuator1 < actuator2:
                        actuator1 = 5
                        actuator2 = 6
                    else:
                        actuator1 = 6
                        actuator2 = 5
                else:
                    body_location = "arm"
                    if actuator1 < actuator2:
                        actuator1 = 7
                        actuator2 = 8
                    else:
                        actuator1 = 8
                        actuator2 = 7
                print(f"Actuators set to {body_location} location")


            elif params[0] in "voltage":
                new_voltage = float(params[1])
                if new_voltage > 2.5:
                    print(f"WARNING: voltage is too high ({new_voltage})")
                else:
                    global voltage
                    voltage = new_voltage
                    print(f"Voltage now set to {voltage}V")

            else:
                print("Unknown command: \n")
                save_params = False
            
            if not loop:
                break

            sleep(3)
    
    except KeyboardInterrupt:
        print("Killing loop")
        for pwmMotor in pwm:
            pwmMotor.ChangeDutyCycle(0)
        
        ser.write('VSET1:0'.encode())
        for thermal_pin in thermal_pins:
            GPIO.output(thermal_pin, GPIO.LOW)

    return save_params

def rabbit(pulse_duration, pulse_interval, hops):
    if hops == 3:
        hops = 3
        pulse(actuator1, pulse_duration, 100)
        sleep(.8)
        pulse(actuator1, pulse_duration, 100)
        sleep(.08)
        pulse(actuator2, pulse_duration, 100)

    else:
        hops = 9
        for _ in range(int(hops / 3 * 2)):
            pulse(actuator1, pulse_duration, 100)
            sleep(pulse_interval)

        for _ in range(int(hops / 3)):
            pulse(actuator2, pulse_duration, 100)
            sleep(pulse_interval)
        

def mask_rabbit(pulse_duration, pulse_interval, hops, warmup):
    temp_polarity = 0
    polarity_pin_1 = thermal_pins[1] if temp_polarity == 1 else thermal_pins[0]
    polarity_pin_2 = thermal_pins[2] if temp_polarity == 1 else thermal_pins[3]
    ser.write(f'VSET1:{voltage}'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.HIGH)
        GPIO.output(polarity_pin_2, GPIO.HIGH)

    sleep(preheat_time)

    if warmup:
        pwm[actuator1].ChangeDutyCycle(100)
        sleep(warmup_time)
        pwm[actuator1].ChangeDutyCycle(0)

    sleep(.5)

    for _ in range(1):
        rabbit(pulse_duration, pulse_interval, hops)
        sleep(0.5)

    ser.write('VSET1:0'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.LOW)
        GPIO.output(polarity_pin_2, GPIO.LOW)


def funnel(duration, location):
    location = float(location)
    intensity1 = 100 * math.sqrt(location / (location + (1 - location)))
    intensity2 = 100 * math.sqrt((1 - location) / (location + (1 - location)))

    pwm[actuator1].ChangeDutyCycle(float(intensity1))
    pwm[actuator2].ChangeDutyCycle(float(intensity2))
    sleep(float(duration))
    pwm[actuator1].ChangeDutyCycle(0)
    pwm[actuator2].ChangeDutyCycle(0)

def mask_funnel(pulse_duration, location, warmup):
    temp_polarity = 1
    polarity_pin_1 = thermal_pins[1] if temp_polarity == 1 else thermal_pins[0]
    polarity_pin_2 = thermal_pins[2] if temp_polarity == 1 else thermal_pins[3]
    global voltage
    ser.write(f'VSET1:{voltage}'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.HIGH)
        GPIO.output(polarity_pin_2, GPIO.HIGH)

    location = float(location)
    intensity1 = 100 * math.sqrt(location / (location + (1 - location)))
    intensity2 = 100 * math.sqrt((1 - location) / (location + (1 - location)))\

    sleep(preheat_time)

    # Warmup
    if warmup:
        pwm[actuator1].ChangeDutyCycle(100)
        pwm[actuator2].ChangeDutyCycle(100)
        sleep(warmup_time)
        pwm[actuator1].ChangeDutyCycle(0)
        pwm[actuator2].ChangeDutyCycle(0)
 
    sleep(1)

    funnel(pulse_duration, location)

    ser.write('VSET1:0'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.LOW)
        GPIO.output(polarity_pin_2, GPIO.LOW)

def mask_motion(pulse_duration, interval, warmup):    
    temp_polarity = 1
    polarity_pin_1 = thermal_pins[1] if temp_polarity == 1 else thermal_pins[0]
    polarity_pin_2 = thermal_pins[2] if temp_polarity == 1 else thermal_pins[3]
    ser.write(f'VSET1:{voltage}'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.HIGH)
        GPIO.output(polarity_pin_2, GPIO.HIGH)

    sleep(preheat_time)

    # Warmup
    if warmup:
        pwm[actuator1].ChangeDutyCycle(100)
        sleep(warmup_time)
        pwm[actuator1].ChangeDutyCycle(0)

    sleep(0.5)

    motion(pulse_duration, interval)

    ser.write('VSET1:0'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.LOW)
        GPIO.output(polarity_pin_2, GPIO.LOW)


def motion(pulse_duration, interval):
    pwm[actuator1].ChangeDutyCycle(75)
    sleep(float(interval))
    pwm[actuator2].ChangeDutyCycle(75)
    sleep(float(pulse_duration) - float(interval))
    pwm[actuator1].ChangeDutyCycle(0)
    sleep(float(interval))
    pwm[actuator2].ChangeDutyCycle(0)

def mask(polarity, thingy):
    activate_thermal(polarity, voltage)
    if thingy == 1:
        pwm[actuator1].ChangeDutyCycle(100)
        sleep(5)
        pwm[actuator1].ChangeDutyCycle(0)
    if thingy == 2:
        pwm[actuator2].ChangeDutyCycle(100)
        sleep(5)
        pwm[actuator2].ChangeDutyCycle(0)
    if thingy == 3:
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

def test():
    print()

def activate_thermal(temp_polarity, voltage):
    polarity_pin_1 = thermal_pins[3] if temp_polarity == 1 else thermal_pins[2]
    polarity_pin_2 = thermal_pins[0] if temp_polarity == 1 else thermal_pins[1]
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