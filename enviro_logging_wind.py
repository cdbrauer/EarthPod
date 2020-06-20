"""
EarthPod Environment Logging with Wind

----

Copyright 2020 - Cole Brauer
"""

import argparse
import itertools
import os
import time
import RPi.GPIO as GPIO

from coral.enviro.board import EnviroBoard
from coral.cloudiot.core import CloudIot
from luma.core.render import canvas
from PIL import ImageDraw
from time import sleep

ENC_PIN = 18
MODE_PIN = 13

DEFAULT_CONFIG_LOCATION = os.path.join(os.path.dirname(__file__), 'cloud_config.ini')

global counter

def update_display(display, msg):
    with canvas(display) as draw:
        draw.text((0, 0), msg, fill='white')

def check_nan(val):
    return float('nan') if val is None else val
    
def increment_counter(channel):
    global counter
    counter += 1

if __name__ == '__main__':
    # Save program start time
    t0 = time.time()
    counter = 0

    # Pull arguments from command line.
    parser = argparse.ArgumentParser(description='Enviro Kit Datalogging')
    parser.add_argument('--filename', help='File for storing sensor data logs', type=str, default=('Log_' + str(time.asctime(time.localtime(time.time()))).replace(' ', '_').replace(':', '-') + '.csv'))
    parser.add_argument('--time_step', help='Measurement logging interval (seconds)', type=int, default=6)
    parser.add_argument('--upload_step', help='Cloud upload interval (seconds)', type=int, default=300)
    parser.add_argument('--cloud_config', help='Cloud IoT config file', default=DEFAULT_CONFIG_LOCATION)
    parser.add_argument('--display', help='Enable printing data to terminal', type=int, default=False)
    parser.add_argument('--wind', help='Enable printing wind rev/sec to terminal', type=int, default=False)
    args = parser.parse_args()

    # Create log file
    f = open(os.path.join(os.path.dirname(__file__), 'logs/') + args.filename, 'w+')
    f.write('EarthPod Sensor Log\n')
    f.write('Created: ' + str(time.asctime(time.localtime(time.time()))) + '\n\n')
    f.write('Measurement Time,Time Step (s),Temperature (C),Humidity (%),Light (lux),Pressure (kPa),Wind (m/s)\n')
    f.close()
    
    # Enable anemometer board
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MODE_PIN, GPIO.OUT)
    GPIO.output(MODE_PIN, False)
    sleep(2)
    GPIO.setup(MODE_PIN, GPIO.IN)
    
    # Initialize anemometer sensor interrupt
    GPIO.setup(ENC_PIN, GPIO.IN)
    GPIO.add_event_detect(ENC_PIN, GPIO.RISING, callback=increment_counter, bouncetime=3)

    # Create instances of EnviroKit and Cloud IoT.
    enviro = EnviroBoard()
    with CloudIot(args.cloud_config) as cloud:
        # Indefinitely update display and upload to cloud.
        sensors = {}
        read_period = int(args.upload_step / args.time_step)

        for read_count in itertools.count():
            # Read sensors
            sensors['time'] = str(time.asctime(time.localtime(time.time())))
            sensors['time_step'] = round(time.time() - t0, 3)
            t0 = time.time()
            sensors['temperature'] = enviro.temperature
            sensors['humidity'] = enviro.humidity
            sensors['ambient_light'] = enviro.ambient_light
            sensors['pressure'] = enviro.pressure
            sensors['wind'] = (counter/sensors['time_step'])*0.053
            counter = 0

            # Save latest data
            f = open(os.path.join(os.path.dirname(__file__), 'logs/') + args.filename, 'a+')
            f.write(str(sensors['time']) + ',')
            f.write(str(sensors['time_step']) + ',')
            f.write(str(sensors['temperature']) + ',')
            f.write(str(sensors['humidity']) + ',')
            f.write(str(sensors['ambient_light']) + ',')
            f.write(str(sensors['pressure']) + ',')
            f.write(str(sensors['wind']) + '\n')
            f.close()

            # Print data to terminal
            if args.display:
                print(str(sensors))

            if args.wind:
                print(str(sensors['wind']) + ' m/s')

            # Display temperature and RH
            msg = 'Wind: %.2f m/s\n' % check_nan(sensors['wind'])
            msg += 'Temp: %.2f C' % check_nan(sensors['temperature'])
            update_display(enviro.display, msg)
            sleep(args.time_step / 3)

            msg = 'RH: %.2f %%\n' % check_nan(sensors['humidity'])
            msg += 'Pressure: %.2f kPa' % check_nan(sensors['pressure'])
            update_display(enviro.display, msg)
            sleep(args.time_step / 3)

            msg = 'Light: %.2f lux\n' % check_nan(sensors['ambient_light'])
            update_display(enviro.display, msg)
            sleep(args.time_step / 3)

            # If time has elapsed, attempt cloud upload.
            if read_count % read_period == 0 and cloud.enabled():
                print('Publishing')
                cloud.publish_message(sensors)