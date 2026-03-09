import RPi.GPIO as GPIO
from time import sleep, time
import sys
import serial
import math
import random
import asyncio

### Tactile ################################################

tactile_pins = [26, 24, 22, 18, 16, 12, 10, 8, 3, 5, 7, 11]
#tactile_pins = [37,35,33,31,29,23,21,19,15]

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

actuator1 = 0
actuator2 = 1
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
preheat_time = 1.25
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

        save_params = asyncio.run(handle_params(params, loop))

        if save_params: 
            prev_prev_params = prev_params
            prev_params = params

async def handle_params(params, loop):
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

            elif params[0] in "set":
                actuator1 = int(params[1])
                actuator2 = int(params[2])
                print(f"Actuators set to {actuator1} and {actuator2}")


            elif params[0] in "voltage":
                new_voltage = float(params[1])
                if new_voltage > 2.5:
                    print(f"WARNING: voltage is too high ({new_voltage})")
                else:
                    global voltage
                    voltage = new_voltage
                    print(f"Voltage now set to {voltage}V")

            elif params[0] in "breathe":
                await breathe()

            elif params[0] in "watch":
                if len(params) < 3:
                    params.append("bruh")
                if "fun" in params[1]:
                    await watch_fun(params[2])
                if "rab" in params[1]:
                    await watch_rab(params[2])
                if "mot" in params[1]:
                    await watch_mot(params[2])
                if "bre" in params[1]:
                    await watch_breathe()
                if "bul" in params[1]:
                    await watch_bulge()
            else:
                print("Unknown command: \n")
                save_params = False
            
            if not loop:
                break

            sleep(3)
    
    except KeyboardInterrupt:
        print("Killing loop")
        deactivate_thermal()
        for pwmMotor in pwm:
            pwmMotor.ChangeDutyCycle(0)
        
        ser.write('VSET1:0'.encode())
        for thermal_pin in thermal_pins:
            GPIO.output(thermal_pin, GPIO.LOW)

    return save_params
    
async def watch_rab_old(thermal_polarity):
    loops = 3
    saltation_duration = 0.08
    saltation_interval = 0.13
    hop_count = 3
    time = 0
    actuator_order = [3,2,1,0]
    async with asyncio.TaskGroup() as tg:
        thermal = 0 if "cold" in thermal_polarity else 1
        tg.create_task(activate_thermal_async(thermal, 1.8, 0))

    sleep(preheat_time)

    async with asyncio.TaskGroup() as tg:
        for loop in range(loops):
            for actuator in actuator_order:
                for _ in range(hop_count):
                    tg.create_task(pulseAsync(actuator, saltation_duration, 100, time))
                    time += saltation_interval
        
    deactivate_thermal()

async def watch_breathe():
    loops = 3
    time = 0
    hop_count = 3
    actuators = [0, 1, 2]
    actuator = 0
    saltation_duration = 0.08
    saltation_interval = 0.13
    motion_duration = 1.2
    motion_interval = 0.5
    funneling_duration = 0.12
    funneling_interval = 0.5
    
    async with asyncio.TaskGroup() as tg:
        for loop in range(loops):
            tg.create_task(activate_thermal_async(1, 1.8, time))
            if loop == 0:
                time += preheat_time
            # BreatheIn - Motion
            tg.create_task(pulseAsync(0, motion_duration / 2, 30, time))
            time += motion_interval / 2
            tg.create_task(pulseAsync(1, motion_duration, 50, time))
            time += motion_interval
            tg.create_task(pulseAsync(2, motion_duration, 65, time, 1))
            time += motion_interval
            tg.create_task(pulseAsync(0, motion_duration, 80, time, 1))
            time += motion_interval

            # BreatheIn - Topoff saltation
            for _ in range(hop_count):
                tg.create_task(pulseAsync(1, saltation_duration, 100, time))
                time += saltation_interval * 1.2
            for _ in range(hop_count):
                tg.create_task(pulseAsync(2, saltation_duration, 100, time))
                time += saltation_interval * 1.4
            for _ in range(hop_count):
                tg.create_task(pulseAsync(0, saltation_duration, 100, time))
                time += saltation_interval * 1.6

            # Thermal switch
            tg.create_task(deactivate_thermal_async(time - 0.1))
            tg.create_task(activate_thermal_async(0, 2.3, time))
            
            # Hold - funneling
            tg.create_task(pulseAsync(0, funneling_duration, 95, time))
            tg.create_task(pulseAsync(1, funneling_duration, 95, time))
            time += funneling_interval * 1.2
            tg.create_task(pulseAsync(1, funneling_duration, 95, time))
            tg.create_task(pulseAsync(2, funneling_duration, 95, time))
            time += funneling_interval * 1.2
            tg.create_task(pulseAsync(2, funneling_duration, 95, time))
            tg.create_task(pulseAsync(0, funneling_duration, 95, time))
            time += funneling_interval * 1.2

            # BreatheOut - Motion
            tg.create_task(pulseAsync(2, motion_duration * 1.2, 50, time))
            time += motion_interval * 1.1
            tg.create_task(pulseAsync(1, motion_duration * 1.2, 65, time, 1))
            time += motion_interval * 1.1
            tg.create_task(pulseAsync(0, motion_duration * 1.4, 80, time, 1))
            time += motion_interval * 1.2
            tg.create_task(pulseAsync(2, motion_duration * 1.4, 65, time, 1))
            time += motion_interval * 1.2
            tg.create_task(pulseAsync(1, motion_duration * 1.4, 65, time, 1))
            time += motion_interval * 1.1
            tg.create_task(pulseAsync(0, motion_duration * 1.2, 65, time, 1))
            time += motion_interval / 2
            tg.create_task(pulseAsync(2, motion_duration / 2, 40, time, 1))

            tg.create_task(deactivate_thermal_async(time + 1.0))
            time += 2.0

    deactivate_thermal()

async def watch_bulge():
    loops = 2
    time = 0
    hop_count = 3
    actuators = [0, 1, 2]
    actuator = 0
    saltation_duration = 0.08
    saltation_interval = 0.13
    motion_duration = 1.2
    motion_interval = 0.5
    funneling_duration = 0.12
    funneling_interval = 0.5
    
    async with asyncio.TaskGroup() as tg:
        for loop in range(loops):
            tg.create_task(activate_thermal_async(1, 1.8, time))
            if loop == 0:
                time += preheat_time
            # BreatheIn - Motion
            tg.create_task(pulseBulge(0, motion_duration, 50, time, 1))
            tg.create_task(pulseBulge(1, motion_duration, 50, time, 1))
            time += motion_interval
            tg.create_task(pulseBulge(0, motion_duration, 70, time))
            tg.create_task(pulseBulge(1, motion_duration, 70, time))
            time += motion_interval
            tg.create_task(pulseBulge(0, motion_duration, 50, time, 0, 1))
            tg.create_task(pulseBulge(1, motion_duration, 50, time, 0, 1))
            tg.create_task(pulseBulge(2, motion_duration, 70, time, 1))
            time += motion_interval
            tg.create_task(pulseBulge(2, motion_duration, 90, time, 0, 1))
            time += motion_interval

            # BreatheIn - Topoff saltation
            for _ in range(hop_count):
                tg.create_task(pulseAsync(0, saltation_duration, 80, time))
                tg.create_task(pulseAsync(1, saltation_duration, 80, time))
                time += saltation_interval * 1.2
            for _ in range(hop_count):
                tg.create_task(pulseAsync(0, saltation_duration, 100, time))
                tg.create_task(pulseAsync(1, saltation_duration, 100, time))
                time += saltation_interval * 1.4
            for _ in range(hop_count):
                tg.create_task(pulseAsync(0, saltation_duration, 60, time))
                tg.create_task(pulseAsync(1, saltation_duration, 60, time))
                tg.create_task(pulseAsync(2, saltation_duration, 100, time))
                time += saltation_interval * 1.6

            # Thermal switch
            tg.create_task(deactivate_thermal_async(time - 0.1))
            tg.create_task(activate_thermal_async(0, 2.3, time))
            
            # Hold - funneling
            tg.create_task(pulseAsync(2, funneling_duration, 95, time))
            time += funneling_interval * 1.2

            tg.create_task(pulseAsync(2, funneling_duration, 95, time))
            time += funneling_interval * 1.2
            tg.create_task(pulseAsync(2, funneling_duration, 95, time))
            time += funneling_interval * 1.2

            # BreatheOut - Motion
            tg.create_task(pulseAsync(2, motion_duration * 1.2, 50, time))
            time += motion_interval * 1.1
            tg.create_task(pulseAsync(1, motion_duration * 1.2, 65, time, 1))
            time += motion_interval * 1.1
            tg.create_task(pulseAsync(0, motion_duration * 1.4, 80, time, 1))
            time += motion_interval * 1.2
            tg.create_task(pulseAsync(2, motion_duration * 1.4, 65, time, 1))
            time += motion_interval * 1.2
            tg.create_task(pulseAsync(1, motion_duration * 1.4, 65, time, 1))
            time += motion_interval * 1.1
            tg.create_task(pulseAsync(0, motion_duration * 1.2, 65, time, 1))
            time += motion_interval / 2
            tg.create_task(pulseAsync(2, motion_duration / 2, 40, time, 1))

            tg.create_task(deactivate_thermal_async(time + 1.0))
            time += 2.0

    deactivate_thermal()

async def watch_fun(thermal_polarity):
    loops = 7
    duration = 0.2
    time = 0

    async with asyncio.TaskGroup() as tg:
        thermal = 0 if "cold" in thermal_polarity else 1
        tg.create_task(activate_thermal_async(thermal, 1.85, 0))

    sleep(preheat_time)

    async with asyncio.TaskGroup() as tg:
        for loop in range(loops):
            actuators = random.sample([0, 1, 2], 2)
            location = random.random()
            intensity1 = 100 * math.sqrt(location / (location + (1 - location)))
            intensity2 = 100 * math.sqrt((1 - location) / (location + (1 - location)))

            tg.create_task(pulseAsync(actuators[0], duration, 100, time))
            tg.create_task(pulseAsync(actuators[1], duration, 100, time))
            time += duration + 0.7
        
    deactivate_thermal()

async def watch_rab(thermal_polarity):
    loops = 3
    saltation_duration = 0.08
    saltation_interval = 0.13
    hop_count = 3
    time = 0
    actuator_order = [1,2,0]
    async with asyncio.TaskGroup() as tg:
        thermal = 0 if "cold" in thermal_polarity else 1
        tg.create_task(activate_thermal_async(thermal, 1.85, 0))

    sleep(preheat_time)

    async with asyncio.TaskGroup() as tg:
        for loop in range(loops):
            for actuator in actuator_order:
                for _ in range(hop_count):
                    tg.create_task(pulseAsync(actuator, saltation_duration, 100, time))
                    time += saltation_interval
        
    deactivate_thermal()

async def watch_mot(thermal_polarity):
    loops = 3
    duration = 0.8
    interval = 0.35
    time = 0
    actuator_order = [1,2,0]
    async with asyncio.TaskGroup() as tg:
        thermal = 0 if "cold" in thermal_polarity else 1
        tg.create_task(activate_thermal_async(thermal, 1.85, 0))

    sleep(preheat_time)

    async with asyncio.TaskGroup() as tg:
        for loop in range(loops):
            for actuator in actuator_order:
                tg.create_task(pulseAsync(actuator, duration, 100, time, 1))
                time += interval
        
    deactivate_thermal()

async def breathe(loops = 3, speed_modifier = 1):
    for loop in range(loops):
        motion_duration = .7
        motion_interval = .3
        saltation_duration = 0.08
        saltation_interval = 0.13
        hop_count = 3
        time = 0
        async with asyncio.TaskGroup() as tg:
            tg.create_task(activate_thermal_async(1, 3.0, 0))

        sleep(preheat_time)

        async with asyncio.TaskGroup() as tg:
            tg.create_task(deactivate_thermal_async(time))
            time += 0.1
            tg.create_task(activate_thermal_async(1, 1.8, time))

            # Motion up
            tg.create_task(pulseAsync(8, motion_duration, 50, time, 1))

            time += motion_interval
            tg.create_task(pulseAsync(6, motion_duration, 50, time, 1))
            tg.create_task(pulseAsync(7, motion_duration, 50, time, 1))
            
            time += motion_interval
            tg.create_task(pulseAsync(2, motion_duration, 65, time, 1))
            tg.create_task(pulseAsync(5, motion_duration, 65, time, 1))

            time += motion_interval
            tg.create_task(pulseAsync(1, motion_duration, 80, time, 1))
            tg.create_task(pulseAsync(4, motion_duration, 80, time, 1))

            #time += motion_duration

            # Saltation        
            time += motion_interval
            for _ in range(hop_count):
                tg.create_task(pulseAsync(2, saltation_duration, 100, time))
                tg.create_task(pulseAsync(5, saltation_duration, 100, time))
                time += saltation_interval * 1.2
            for _ in range(hop_count):
                tg.create_task(pulseAsync(1, saltation_duration, 100, time))
                tg.create_task(pulseAsync(4, saltation_duration, 100, time))
                time += saltation_interval * 1.4
            for _ in range(hop_count):
                tg.create_task(pulseAsync(0, saltation_duration, 100, time))
                tg.create_task(pulseAsync(3, saltation_duration, 100, time))
                time += saltation_interval * 1.6

            for _ in range(3):
                tg.create_task(pulseAsync(0, saltation_duration, 100, time))
                tg.create_task(pulseAsync(3, saltation_duration, 100, time))
                time += saltation_interval * 1.8

            tg.create_task(deactivate_thermal_async(time - 1.0))
            tg.create_task(activate_thermal_async(0, 3.5, time - 0.9))
            tg.create_task(deactivate_thermal_async(time - 0.4))
            tg.create_task(activate_thermal_async(0, 2.3, time))

            # Saltation        
            time += motion_interval
            for _ in range(hop_count):
                tg.create_task(pulseAsync(0, saltation_duration, 100, time))
                tg.create_task(pulseAsync(3, saltation_duration, 100, time))
                time += saltation_interval * 1.2
            for _ in range(hop_count):
                tg.create_task(pulseAsync(1, saltation_duration, 100, time))
                tg.create_task(pulseAsync(4, saltation_duration, 100, time))
                time += saltation_interval * 1.4
            for _ in range(hop_count):
                tg.create_task(pulseAsync(2, saltation_duration, 100, time))
                tg.create_task(pulseAsync(5, saltation_duration, 100, time))
                time += saltation_interval * 1.6

            time -= saltation_interval * 1.4
            # Motion up
            tg.create_task(pulseAsync(1, motion_duration * 1.4, 60, time, 1))
            tg.create_task(pulseAsync(4, motion_duration * 1.4, 60, time, 1))
            
            time += motion_interval * 1.2
            tg.create_task(pulseAsync(2, motion_duration * 1.4, 50, time, 1))
            tg.create_task(pulseAsync(5, motion_duration * 1.4, 50, time, 1))

            time += motion_interval * 1.2
            tg.create_task(pulseAsync(6, motion_duration * 1.4, 40, time, 1))
            tg.create_task(pulseAsync(7, motion_duration * 1.4, 40, time, 1))


        deactivate_thermal()
        sleep(0.2)
    deactivate_thermal()

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
    temp_polarity = 1
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
    intensity2 = 100 * math.sqrt((1 - location) / (location + (1 - location)))

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
    activate_thermal(polarity, 1.8)
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
    polarity_pin_1 = thermal_pins[3] if temp_polarity == 0 else thermal_pins[2]
    polarity_pin_2 = thermal_pins[0] if temp_polarity == 0 else thermal_pins[1]
    ser.write(f'VSET1:{voltage}'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.HIGH)
        GPIO.output(polarity_pin_2, GPIO.HIGH)

async def activate_thermal_async(temp_polarity, voltage, waitTime):
    await asyncio.sleep(waitTime)
    polarity_pin_1 = thermal_pins[3] if temp_polarity == 0 else thermal_pins[2]
    polarity_pin_2 = thermal_pins[0] if temp_polarity == 0 else thermal_pins[1]
    ser.write(f'VSET1:{voltage}'.encode())
    if temp_polarity != None:
        GPIO.output(polarity_pin_1, GPIO.HIGH)
        GPIO.output(polarity_pin_2, GPIO.HIGH)

async def activate_thermal_task(temp_polarity, voltage, waitTime):
    return asyncio.create_task(temp_polarity, voltage, waitTime)

def deactivate_thermal():
    ser.write('VSET1:0'.encode())
    for thermal_pin in thermal_pins:
        GPIO.output(thermal_pin, GPIO.LOW)

async def deactivate_thermal_async(waitTime):
    await asyncio.sleep(waitTime)
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

async def pulseAsync(pwmIndex, duration, intensity, waitTime, ramp = 0):
    await asyncio.sleep(waitTime)
    
    if ramp == 1:
        rampWindow = duration / 3
        rampStep = intensity / 3
        pwm[pwmIndex].ChangeDutyCycle(rampStep)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 2)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 3)
        await asyncio.sleep(rampWindow / 3)

        await asyncio.sleep(rampWindow)

        pwm[pwmIndex].ChangeDutyCycle(rampStep * 3)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 2)
        await asyncio.sleep(rampWindow / 3)
        pwm[pwmIndex].ChangeDutyCycle(rampStep * 1)
        await asyncio.sleep(rampWindow / 3)

        pwm[pwmIndex].ChangeDutyCycle(0)

    else:
        pwm[pwmIndex].ChangeDutyCycle(intensity)
        await asyncio.sleep(duration)
        pwm[pwmIndex].ChangeDutyCycle(0)

async def pulseBulge(pwmIndex, duration, intensity, waitTime, rampUp = 0, rampDown = 0):
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

async def pulseTask(pwmIndex, duration, intensity, waitTime):
    return asyncio.create_task(pulseAsync(pwmIndex, duration, intensity, waitTime))

main()