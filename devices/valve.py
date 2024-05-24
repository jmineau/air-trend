#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
valve.py switches the solenoid valve between the reference tank and atmosphere.
Usage:
    python valve.py atm
    python valve.py ref
"""

import argparse
import logging
import os
import RPi.GPIO as GPIO


VALVEPIN = int(os.getenv('VALVEPIN', 16))  # default pi pin that controls valve


class Valve:
    """
    A class representing a valve that controls the flow of gas in a gas sensor system.

    Attributes:
    - source (str): The source of the gas flow, either 'atm' for atmosphere or 'ref' for reference.
    - pin (int): The pin number on the Raspberry Pi GPIO board that controls the valve.

    """

    def __init__(self, source='atm', pin=VALVEPIN):

        self.source = source  # valve source
        self.pin = int(pin)  # pi pin that controls valves
        
        # Log valve under flow logger
        self.logger = logging.getLogger('flow')

        # Setup R-Pi board
        self.logger.info('Initializing valve...')
        GPIO.setmode(GPIO.BOARD)  # Set GPIO to use BOARD mode

        # Set GPIO to be a transmitting signal with inital state
        GPIO.setup(self.pin, GPIO.OUT, initial=self.pin_state)

    @property
    def pin_state(self):
        """
        Returns the pin state based on the current source.

        Returns:
        - int: The pin state, either 0 for reference or 1 for atmosphere.
        """
        # 0 if reference (ref)
        # 1 if atmosphere (atm)
        if str(self.source) in ['atm', 'atmosphere', '1']:
            self.logger.debug('Setting valve to atmosphere!')
            return 1
        elif str(self.source) in ['ref', 'reference', '0']:
            self.logger.debug('Setting valve to reference!')
            return 0
        else:
            raise ValueError('Invalid source!')

    def update(self, source):
        """
        Updates the source of the gas flow and sets the pin state accordingly.

        Args:
        - source (str): The new source of the gas flow, either 'atm' for atmosphere or 'ref' for reference.
        """
        self.source = source
        GPIO.output(self.pin, self.pin_state)


if __name__ == '__main__':

    # Turn off warnings
    GPIO.setwarnings(False)

    # Parse command-line args
    parser = argparse.ArgumentParser(description='Set valve')
    parser.add_argument('source', default='atm', type=str, help='atm or ref')
    parser.add_argument('-pin', '-p', default=VALVEPIN, type=int,
                        help='Raspberry Pi pin that controls valve')

    args = parser.parse_args()  # Parse args

    Valve(args.source, args.pin)
