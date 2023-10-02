#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 13 23:55:32 2023

@author: James Mineau (James.Mineau@utah.edu)
"""

import csv
import logging
import json
from multiprocessing import Process, Queue
import os
import time

import devices

CONFIG = os.getenv("CONFIG", "config.json")
LOGLEVEL = os.getenv("LOGLEVEL", "INFO")

logging.basicConfig(
    level=LOGLEVEL,
    format="%(asctime)s | %(name)s | %(levelname)s | %(message)s")
logger = logging.getLogger()

with open(CONFIG, "r") as f:
    DEVICES = json.load(f)

SERVICE_EXIT = object()


class SerialDevicePoolError(Exception):
    pass


def worker(q: Queue, device_config: dict):
    """Instantiates and indefinitely reads from serial device"""

    Device = getattr(devices, device_config.pop('class', 'SerialDevice'))
    name = device_config.get('name', Device.name)

    if device_config.pop('is_active', True):
        logger.info(f"Starting {name} device worker: {device_config}")
    else:
        logger.info(f"Device {name} currently disabled: {device_config}")
        return None

    device = None
    while True:
        try:
            device = Device(**device_config)
            for data in device.get_data():
                if data:
                    q.put(data)

        except (OSError, KeyboardInterrupt, SystemExit) as e:
            q.put(SERVICE_EXIT)
            raise e

        except BaseException as e:
            logger.exception(e)
            time.sleep(1)

        if device is not None:
            device.close()
            device = None


def writer(q: Queue):
    """Receives messages from queue and writes to file"""
    while True:
        try:
            data = q.get()

            if data is SERVICE_EXIT:
                raise SerialDevicePoolError("Restarting device pool")

            path_template = data.path_template
            record = data.record

            # Format time
            time = record['timestamp']
            record['timestamp'] = time.isoformat()

            path = time.strftime(path_template)
            should_write_header = not os.path.exists(path)

            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "a") as f:
                writer = csv.DictWriter(f, fieldnames=record.keys())
                if should_write_header:
                    writer.writeheader()
                writer.writerow(record)

        except (SerialDevicePoolError, KeyboardInterrupt, SystemExit):
            raise


def main():
    site = os.environ.get('HOSTNAME')
    devices = DEVICES[site]

    q = Queue()

    for device_config in devices:
        p = Process(target=worker, kwargs={"q": q,
                                           "device_config": device_config})
        p.daemon = True
        p.start()

    p = Process(target=writer, kwargs={"q": q})
    p.start()
    p.join()


if __name__ == "__main__":
    main()
