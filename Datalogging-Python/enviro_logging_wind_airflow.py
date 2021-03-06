"""
EarthPod Environment Logging with Wind

----

Copyright 2020 - Cole Brauer
"""

import argparse
import os
import time
import serial
import RPi.GPIO as GPIO

from coral.enviro.board import EnviroBoard
from luma.core.render import canvas
from time import sleep

ENC_PIN = 18
MODE_PIN = 13

global counter

def press_anemometer_mode(duration: float):
    """
    Simulate pressing the anemometer mode button.

    :param duration: Duration of press in seconds
    :return: None
    """
    GPIO.setup(MODE_PIN, GPIO.OUT)
    GPIO.output(MODE_PIN, False)
    sleep(duration)
    GPIO.setup(MODE_PIN, GPIO.IN)

def update_display(display, message: str):
    """
    Update environmental sensor board display.

    :param display: Display object
    :param message: Message string
    :return: None
    """
    with canvas(display) as draw:
        draw.text((0, 0), message, fill='white')

def check_nan(val: float):
    """
    If value does not exist, return nan.

    :param val: Value to check
    :return: Value or nan
    """
    return float('nan') if val is None else val
    
def increment_counter(channel):
    """
    Callback function to increment the global counter variable.

    :param channel: Interrupt channel
    :return: None
    """
    global counter
    counter += 1

def read_airflow_sensor(message: list):
    recv_error = 0

    msgOut = bytearray(message[0:4])
    ser.write(msgOut)

    msgIn = ser.read(4)
    if len(msgIn) < 4:
        print('Warning: ' + str(len(msgIn)) + '/4 bytes received')
        recv_error = 1

    return int.from_bytes(msgIn[0:2], byteorder='big'), recv_error

if __name__ == '__main__':
    print('Initializing...')

    # Save program start time
    t0 = time.time()
    counter = 0

    # Pull arguments from command line.
    parser = argparse.ArgumentParser(description='Enviro Kit Datalogging')
    parser.add_argument('--filename', help='File for storing sensor data logs', type=str, default=('Log_' + str(time.asctime(time.localtime(time.time()))).replace(' ', '_').replace(':', '-') + '.csv'))
    parser.add_argument('--time_step', help='Measurement logging interval (seconds)', type=int, default=6)
    parser.add_argument('--display', help='Enable printing data to terminal', type=int, default=False)
    parser.add_argument('--wind', help='Enable printing wind rev/sec to terminal', type=int, default=False)
    args = parser.parse_args()

    # Create log file
    f = open(os.path.join(os.path.dirname(__file__), 'logs/') + args.filename, 'w+')
    f.write('EarthPod Sensor Log\n')
    f.write('Created: ' + str(time.asctime(time.localtime(time.time()))) + '\n\n')
    f.write('Measurement Time,Time Step (s),Temperature (C),Humidity (%),Light (lux),Pressure (kPa),Interrupt Counter (counts/time step),Wind (m/s),Airflow (m/s),Airflow Ambient Temp (C),Airflow Error\n')
    f.close()
    
    # Enable anemometer board
    GPIO.setmode(GPIO.BCM)
    press_anemometer_mode(2)
    
    # Initialize anemometer sensor interrupt
    print('Enabling anemometer...')
    GPIO.setup(ENC_PIN, GPIO.IN)
    GPIO.add_event_detect(ENC_PIN, GPIO.RISING, callback=increment_counter, bouncetime=3)

    # Create instances of EnviroKit and Cloud IoT.
    enviro = EnviroBoard()

    # Open serial connection
    ser = serial.Serial('/dev/ttyAMA0', 19200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=2)

    # Done with initialization
    print('Done')

    # Indefinitely update display and log data
    while True:
        # Read built-in sensors
        sensors = {'time': str(time.asctime(time.localtime(time.time()))),
                   'time_step': round(time.time() - t0, 3),
                   'count':counter,
                   'temperature': enviro.temperature,
                   'humidity': enviro.humidity,
                   'ambient_light': enviro.ambient_light,
                   'pressure': enviro.pressure}

        # Reset timer and counter
        t0 = time.time()
        counter = 0

        # Get anemometer reading
        sensors['wind'] = (sensors['count']/sensors['time_step'])*0.053

        # Read airflow sensor
        airflow, error = read_airflow_sensor([1, 0, 0, 1^0^0])
        sensors['airflow'] = airflow/1000.0
        sensors['airflowError'] = error

        t_ambient, error = read_airflow_sensor([2, 0, 0, 2^0^0])
        sensors['airflowAmbTemp'] = t_ambient/100.0
        sensors['airflowError'] = sensors['airflowError'] or error

        # Keep anemometer awake
        press_anemometer_mode(0.1)

        # Save latest data
        f = open(os.path.join(os.path.dirname(__file__), 'logs/') + args.filename, 'a+')
        f.write(str(sensors['time']) + ',')
        f.write(str(sensors['time_step']) + ',')
        f.write(str(sensors['temperature']) + ',')
        f.write(str(sensors['humidity']) + ',')
        f.write(str(sensors['ambient_light']) + ',')
        f.write(str(sensors['pressure']) + ',')
        f.write(str(sensors['count']) + ',')
        f.write(str(sensors['wind']) + ',')
        f.write(str(sensors['airflow']) + ',')
        f.write(str(sensors['airflowAmbTemp']) + ',')
        f.write(str(sensors['airflowError']) + '\n')
        f.close()

        # Print data to terminal
        if args.display:
            print(str(sensors))
        elif args.wind:
            print(str(sensors['wind']) + ' m/s, ' + str(sensors['airflow']) + 'm/s')

        # Display temperature and RH
        msg = 'Wind: %.2f m/s\n' % check_nan(sensors['wind'])
        msg += 'Airflow: %.2f m/s' % check_nan(sensors['airflow'])
        update_display(enviro.display, msg)
        sleep(args.time_step / 3)

        msg = 'Temp: %.2f C\n' % check_nan(sensors['temperature'])
        msg += 'RH: %.2f %%' % check_nan(sensors['humidity'])
        update_display(enviro.display, msg)
        sleep(args.time_step / 3)

        msg = 'Pressure: %.2f kPa\n' % check_nan(sensors['pressure'])
        msg += 'Light: %.2f lux' % check_nan(sensors['ambient_light'])
        update_display(enviro.display, msg)
        sleep(args.time_step / 3)
