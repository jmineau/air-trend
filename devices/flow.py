#!/usr/bin/python
# Watches status.py file and changes GPIO settings for
# solenoid valves and calibration systems.
# Code originally by Ben Fasoli, written in Python 2
# Updated by James Mineau, 2022-09-29, converted to Python 3 & added comments
# Renamed from controlValves to flow, imports valve.py - JM 2022-10-31
# Switched from using real time to control valves to specifying durations - JM 2023-07-13
# ----------------------------------------------------------------------------

from multiprocessing import Process
import RPi.GPIO as GPIO
import time

from valve import Valve
from logger import Logger

ID_PATH = '/home/pi/.valve.id'

IDs = {
    'atmosphere': 1,
    'reference': 2
}

logger = Logger('flow').logger


def wait(h=0, m=0, s=1):
    seconds = (h * 60 * 60) + (m * 60) + s
    time.sleep(seconds)


class FlowControlSystem:
    def __init__(self, valve_pin):
        # Initialize Valve Control System
        self.valve = Valve(pin=valve_pin)
        self.ID = None

        self.process = Process(target=self.flow)
        self.process.daemon = True

    def update(self, source, ID):
        self.valve.update(source)
        self.ID = ID

    def flush(self, source, h=0, m=0, s=90):
        ID = -1 * IDs[source]  # Get ID

        logger.info(f'Flushing - Setting ID: {ID}')
        self.update(source, ID)
        wait(h, m, s)

    def measure(self, source, h=0, m=0, s=0):
        if (h + m + s) == 0:
            raise ValueError('Measurement time cannot be 0!')

        # Flush for 90 seconds before measuring
        self.flush(source)

        ID = IDs[source]  # Get ID

        logger.info(f'Measuring {source.capitalize()} - Setting ID: {ID}')
        self.update(source, ID)
        wait(h, m, s)

    def flow(self):
        try:
            while True:
                # Measure atmosphere for 1 hour
                self.measure('atmosphere', h=1)

                # Measure reference tank for 90 seconds
                self.measure('reference', s=90)

        except BaseException:
            self.cleanup()
            raise

    def start(self):
        self.process.start()

    def cleanup(self):
        logger.info('Cleaning up...')
        self.valve.update('atmosphere')  # End with atmospheric valve
        GPIO.cleanup()  # cleanup pi pins


if __name__ == '__main__':
    FlowControlSystem().initialize()
