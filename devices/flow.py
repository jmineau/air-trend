#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
from multiprocessing import Process, Array
import RPi.GPIO as GPIO
import time

from valve import Valve


# Sources of gas for the flow control system
sources = [
    'atmosphere',
    'reference',
    'flush'
]


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
    A class representing a flow control system that switches between different gas sources.

    Attributes:
    - valve_pin (int): The raspberry GPIO board pin number for the valve control system.
    - valve (Valve): The valve control system.
    - source (str): The source of the gas flow.
    """

    def __init__(self, valve_pin=None):
        # Initialize Valve Control System
        self.valve = Valve(pin=valve_pin)
        self._source = Array('c', b' ' * max(len(s) for s in sources))

        self._process = Process(target=self.flow)
        self._process.daemon = True
        
        self.logger = logging.getLogger(__name__)

    @property
    def source(self):
        """
        Get the source of the gas flow.
        """
        return self._source.value.decode('utf-8').strip()

    @source.setter
    def source(self, source):
        """
        Set the source of the gas flow.
        """
        self._source.value = source.encode('utf-8')

    def flush(self, h=0, m=0, s=90):
        """
        Flushes the valve control system for a given time.

        Args:
        - h (int): The hours to flush the valve control system (default 0).
        - m (int): The minutes to flush the valve control system (default 0).
        - s (int): The seconds to flush the valve control system (default 90).
        """

        self.logger.debug(f'Flushing...')
        self.source = 'flush'
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

        # Update valve to source
        self.valve.update(source)

        # Flush for 90 seconds before measuring
        self.flush(s=flush)

        # Measure source
        self.logger.debug(f'Measuring {source.capitalize()}...')
        self.source = source
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
        self.source = 'atmosphere'
        self.valve.update('atmosphere')  # End with atmospheric valve
        GPIO.cleanup()  # cleanup pi pins


if __name__ == '__main__':
    FlowControlSystem().start()
