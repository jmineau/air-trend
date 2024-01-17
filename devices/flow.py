#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
from multiprocessing import Process
import RPi.GPIO as GPIO
import time

from valve import Valve

ID_PATH = '/home/pi/.valve.id'

IDs = {
    'atmosphere': 1,
    'reference': 2
}


import time

def wait(h=0, m=0, s=1):
    """
    Wait for a specified amount of time.

    Args:
        h (int): Hours to wait (default is 0).
        m (int): Minutes to wait (default is 0).
        s (int): Seconds to wait (default is 1).
    """
    seconds = (h * 60 * 60) + (m * 60) + s
    time.sleep(seconds)


class FlowControlSystem:
    """
    A class representing a flow control system that switches between atmosphere and reference gas.

    Attributes:
    - valve_pin (int): The raspberry GPIO board pin number for the valve control system.
    - valve (Valve): The valve control system.
    - ID (str): The ID of the current state of the flow control system.
    """

    def __init__(self, valve_pin):
        # Initialize Valve Control System
        self.valve = Valve(pin=valve_pin)
        self.ID = None

        self._process = Process(target=self.flow)
        self._process.daemon = True
        
        self.logger = logging.getLogger(__name__)

    def update(self, source):
        """
        Updates the valve control system with the given source.

        Args:
        - source (str): The source of the gas.
        """
        self.valve.update(source)
        self.ID = source

    def flush(self, h=0, m=0, s=90):
        """
        Flushes the valve control system for a given time.

        Args:
        - h (int): The hours to flush the valve control system (default 0).
        - m (int): The minutes to flush the valve control system (default 0).
        - s (int): The seconds to flush the valve control system (default 90).
        """

        self.logger.debug(f'Flushing')
        self.ID = 'flush'
        wait(h, m, s)

    def measure(self, source, h=0, m=0, s=0, flush=90):
        """
        Measures the gas for a given source and time.

        Args:
        - source (str): The source of the gas.
        - h (int): The hours to measure (default 0).
        - m (int): The minutes to measure (default 0).
        - s (int): The seconds to measure (default 0).
        - flush (int): The seconds to flush before measuring (default 90).
        """
        if (h + m + s) == 0:
            raise ValueError('Measurement time cannot be 0!')

        # Flush for 90 seconds before measuring
        self.flush(s=flush)

        self.logger.debug(f'Measuring {source.capitalize()}')
        self.update(source)
        wait(h, m, s)

    def flow(self):
        """
        Starts the flow control system and measures atmospheric and reference tank gas.
        """
        self.logger.info('Starting flow control system...')
        
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
        """
        Starts the flow control system process.
        """
        self._process.start()

    def cleanup(self):
        """
        Cleans up the flow control system by updating the valve to atmosphere and cleaning up the GPIO pins.
        """
        self.logger.debug('Cleaning up...')
        self.update('atmosphere')  # End with atmospheric valve
        GPIO.cleanup()  # cleanup pi pins


if __name__ == '__main__':
    FlowControlSystem().start()
