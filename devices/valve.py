#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 31 18:51:07 2022

@author: James Mineau

valve.py switches the solenoid valve between the reference tank and atmosphere.
Usage:
    python valve.py atm
    python valve.py ref
"""

import argparse
import os

import RPi.GPIO as GPIO

from logger import Logger

VALVEPIN = int(os.getenv('VALVEPIN', 16))  # default pi pin that controls valve

logger = Logger('flow').logger


class Valve:
    def __init__(self, source='atm', pin=VALVEPIN):
        self.source = source  # valve source
        self.pin = int(pin)  # pi pin that controls valves

        # Setup R-Pi board
        logger.info('Activating valve control system...')
        GPIO.setmode(GPIO.BOARD)  # Set GPIO to use BOARD mode

        # Set GPIO to be a transmitting signal with inital state
        GPIO.setup(self.pin, GPIO.OUT, initial=self.pin_state)

    @property
    def pin_state(self):
        # 0 if reference (ref)
        # 1 if atmosphere (atm)
        if str(self.source) in ['atm', 'atmosphere', '1']:
            logger.info('Setting valve to atmosphere!')
            return 1
        elif str(self.source) in ['ref', 'reference', '0']:
            logger.info('Setting valve to reference!')
            return 0
        else:
            raise ValueError('Invalid source!')

    def update(self, source):
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
