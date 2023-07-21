#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from datetime import datetime as dt
import logging
import os
import re
import string
import time
from typing import Iterator, List, Union

import serial

# TODO :
    # update docs about filter_response
    # GPS GPRMC handler
    # subclasses for wbb instruments
    # time or get_data first?

DATAPATH = os.getenv('DATAPATH', 'data')


def serial_port(port: str, parent: str = '/dev/serial/by-id'):
    # TODO docstr
    return os.path.join(parent, port)


class Data:
    # TODO docstr
    def __init__(self,
                 path_template: str,
                 record: dict):

        self.path_template = path_template
        self.record = record


class ResponseHandler:
    # TODO docstr
    def __init__(self,
                 path_template: str,
                 variables: List[dict],
                 line_filters: Union[str, List[str]] = None,
                 variable_filters: Union[str, dict] = None,
                 delimiter: str = ','):

        self.path_template = path_template
        self.variables = variables
        self.delimiter = delimiter
        self.n_variables = len(variables)

        if line_filters is not None and not isinstance(line_filters, list):
            line_filters = [line_filters]
        self.line_filters = line_filters

        if (variable_filters is not None
                and not isinstance(variable_filters, dict)):

            variable_filters = {variable['name']: variable_filters
                                for variable in variables}
        self.variable_filters = variable_filters

    def match_line(self, response: str) -> bool:
        # TODO docstr
        if self.line_filters:
            return any(re.search(filter_string, response)
                       for filter_string in self.line_filters)
        return True

    def format_response(self, response: str, logger: logging.Logger
                        ) -> dict:
        # TODO docstr
        logger.debug('Formatting response: ', response)  # FIXME

        if not self.match_line(response):
            raise ValueError(f'Handler does not match response: {response}')

        row = response.split(self.delimiter)

        for i, variable in enumerate(row):
            row[i] = variable.strip()

        if self.variable_filters:
            for i, variable in enumerate(row):
                variable_name = self.variables[i]['name']
                filter_string = self.variable_filters.get(variable_name)
                if filter_string:
                    match = re.search(filter_string, variable)
                    if match:
                        row[i] = match.group()
                else:
                    row[i] = variable

        response = self.delimiter.join(row)
        response = ''.join([s for s in response if s in string.printable])

        logger.debug('Formatted response: ', response)  # FIXME

        variables = re.split(self.delimiter, response)

        n_var = len(variables)
        if not n_var == self.n_variables:
            logger.debug(f'Found {n_var} records '
                         f'but config specifies {self.n_variables}.')
            return None

        record = {self.variables[i]['name']: variables[i]
                  for i in range(n_var)
                  if self.variables[i]['save']}

        return record


class SerialDevice:
    # TODO docstr
    def __init__(self,
                 name: str,
                 port: str,
                 baudrate: int = 9600,
                 variables: List[dict] = None,
                 handlers: list = None,
                 delimiter: str = ',',
                 eol_delimiter: str = '\r\n',
                 line_filters: Union[str, List[str]] = None,
                 variable_filters: Union[str, dict] = None,
                 init_command: str = None,
                 poll_command: str = None,
                 poll_interval: float = 0,
                 poll_type: str = None,
                 timeout: float = 60):

        self.name = name.strip()
        self.port = port
        self.baudrate = baudrate
        self.delimiter = delimiter
        self.eol_delimiter = eol_delimiter
        self.poll_command = poll_command
        self.poll_interval = poll_interval
        self.poll_type = poll_type
        self.timeout = timeout

        self.logger = logging.getLogger(name)

        if handlers is None:
            # Build device handler
            path_template = os.path.join(DATAPATH, name, '%Y-%m-%d.csv')
            handler = ResponseHandler(path_template, variables,
                                      line_filters, variable_filters,
                                      delimiter)
            handlers = [handler]
        self.handlers = handlers

        # Encode
        if eol_delimiter:
            self.eol_delimiter = eol_delimiter.encode('utf-8')
        if poll_command:
            self.poll_command = poll_command.encode('utf-8')

        # Establish connection with serial device
        self.connection = serial.Serial(self.port, baudrate, timeout=timeout)

        if init_command:
            # Send init command to device
            self.connection.write(init_command.encode('utf-8'))

    def _data_generator(self, timestamp, response):
        # TODO: docstr
        try:
            response = response.decode('utf-8')
        except UnicodeDecodeError:
            return None

        matching_handlers = [h for h in self.handlers
                             if h.match_line(response)]

        for handler in matching_handlers:
            record = handler.format_response(response, self.logger)
            data = Data(handler.path_template, {'timestamp': timestamp,
                                                **record})
            yield data

    def _poll_batch(self) -> Iterator[dict]:
        '''Polls device for data returned and optionally formatted using regex

        Generator that periodically polls device, waiting poll_interval seconds
        and then reading all available bytes from the serial buffer.
        Optionally,filter_response can be used to match and filter the returned
        data. Yields list of dictionary records.
        '''
        while True:
            self.connection.write(self.poll_command)
            timestamp = dt.utcnow()
            time.sleep(self.poll_interval)

            response = b''
            while self.connection.in_waiting:
                response = response + self.connection.read()

            yield from self._data_generator(timestamp, response)

    def _poll_line(self) -> Iterator[dict]:
        '''Polls device for preformatted line observation set

        Generator that periodically polls device then blocks until a full line
        of data is received to the serial buffer. Yields list of dictionary
        records.
        '''
        while True:
            self.connection.write(self.poll_command)
            timestamp = dt.utcnow()
            time.sleep(self.poll_interval)

            response = self.connection.read_until(self.eol_delimiter)

            yield from self._data_generator(timestamp, response)

    def _stream(self) -> Iterator[dict]:
        '''Returns data from serial buffer

        Generator that blocks until line is received in device's serial buffer
        and yields list of dictionary records.
        '''
        while True:
            response = self.connection.read_until(self.eol_delimiter)
            timestamp = dt.utcnow()

            yield from self._data_generator(timestamp, response)

    def get_data(self) -> Iterator[dict]:
        '''Interface for worker to receive data

        Can be overwritten by subclasses to provide additional complexity.
        Subclasses will need to provide the functionality of _data_generator.
        '''
        if self.poll_type == 'line':
            yield from self._poll_line()
        elif self.poll_type == 'batch':
            yield from self._poll_batch()
        elif self.poll_type is not None:
            raise ValueError('poll_type must be line or batch')
        else:
            yield from self._stream()

    def close(self):
        if self.connection and self.connection.is_open:
            self.connection.flush()
            self.connection.close()


#       ***********
#       INSTRUMENTS
#       ***********


class ADI_MAGIC(SerialDevice):

    # TODO docstr
    """MAGIC: Moderated Aerosol Growth with Internal water Cycling

    https://aerosol.us/magic
    """
    name = 'magic_200p_adi',

    baudrate = 115200
    init_command = 'ppw,215\r\n'

    variables = [
      {'name': 'mod_set',      'save': True},
      {'name': 'mod_t_c',      'save': True},
      {'name': 'input_rh_pct', 'save': True},
      {'name': 'input_t_c',    'save': True},
      {'name': 'input_td_c',   'save': True},
      {'name': 'flow_p_raw',   'save': False},
      {'name': 'flow_p_pa',    'save': True},
      {'name': 'flow_ccmm',    'save': True}
    ]

    def __init__(self, port):
        super().__init__(ADI_MAGIC.name, serial_port(port),
                         baudrate=ADI_MAGIC.baudrate,
                         variables=ADI_MAGIC.variables,
                         init_command=ADI_MAGIC.init_command)


class BB_205(SerialDevice):
    name = '2b_205'

    baudrate = 4800

    variables = [
      {'name': 'o3_ppb',    'save': True},
      {'name': 't_c',       'save': True},
      {'name': 'p_hpa',     'save': True},
      {'name': 'flow_ccpm', 'save': True},
      {'name': 'inst_date', 'save': False},
      {'name': 'inst_time', 'save': False}
    ]

    def __init__(self, port):
        super().__init__(BB_205.name, serial_port(port),
                         variables=BB_205.variables,
                         baudrate=BB_205.baudrate)


class GPS(SerialDevice):
    name = 'gps'

    baudrate = 4800
    data_path = os.path.join(DATAPATH, name)

    # GPGGA handler
    gpgga_path_template = os.path.join(data_path, 'gpgga', '%Y-%m-%d.csv')
    gpgga_variables = [
        {'name': 'nmea',                  'save': False},
        {'name': 'time_gps',              'save': True},
        {'name': 'latitude_dm',           'save': True},
        {'name': 'latitude_ns',           'save': True},
        {'name': 'longitude_dm',          'save': True},
        {'name': 'longitude_ew',          'save': True},
        {'name': 'fix_quality',           'save': True},
        {'name': 'n_sat',                 'save': True},
        {'name': 'horizontal_dilution',   'save': False},
        {'name': 'altitude_amsl',         'save': True},
        {'name': 'altitude_amsl_unit',    'save': False},
        {'name': 'geoidal_separation',    'save': False},
        {'name': 'time_last_update',      'save': False},
        {'name': 'time_last_update_unit', 'save': False},
        {'name': 'stid_and_checksum',     'save': False}]

    # GPRMC handler
    gprmc_path_template = os.path.join(data_path, 'gprmc', '%Y-%m-%d.csv')
    gprmc_variables = [
        {'name': 'nmea',         'save': True},
        {'name': 'time_gps',     'save': True},
        {'name': 'status',       'save': True},
        {'name': 'latitude_dm',  'save': True},
        {'name': 'n_s:',         'save': False},
        {'name': 'longitude_dm', 'save': True},
        {'name': 'e_w',          'save': False},
        {'name': 'speed_kt',     'save': True},
        {'name': 'true_course',  'save': True},
        {'name': 'date_gps',     'save': True},
        {'name': 'variation',    'save': False},
        {'name': 'checksum',     'save': False}]

    # Create handlers
    GPGGA = ResponseHandler(gpgga_path_template, gpgga_variables,
                            line_filters='GPGGA')
    GPRMC = ResponseHandler(gprmc_path_template, gprmc_variables,
                            line_filters='GPRMC')

    def __init__(self, port):
        super().__init__(GPS.name, serial_port(port), baudrate=GPS.baudrate,
                         handlers=[GPS.GPGGA, GPS.GPRMC])


class LGR(SerialDevice):
    # TODO docstr
    from flow import FlowControlSystem

    name = 'lgr_ugga'

    baudrate = 115200
    line_filter = '/'  # match with forward slash in timestamp_LGR

    variables = ['timestamp_lgr', 'CH4_ppm',   'CH4_ppm_sd',  'H2O_ppm',
                 'H2O_ppm_sd',    'CO2_ppm',   'CO2_ppm_sd',  'CH4d_ppm',
                 'CH4d_ppm_sd',   'CO2d_ppm',  'CO2d_ppm_sd', 'GasP_torr',
                 'GasP_torr_sd',  'GasT_C',    'GasT_C_sd',   'AmbT_C',
                 'AmbT_C_sd',     'RD0_us',    'RD0_us_sd',   'RD1_us',
                 'RD1_us_sd',     'Fit_Flag',  'MIU_Valve',   'MIU_Desc']
    variables = [{'name': name, 'save': True} for name in variables]

    path_template = os.path.join(DATAPATH, name, 'new', '%Y-%m-%d.csv')
    handler = ResponseHandler(path_template, variables,
                              line_filters=line_filter)

    def __init__(self, port, valve_pin=None, MIU=False):

        if (bool(valve_pin) + bool(MIU)) != 1:
            raise ValueError('One and only one of valve_pin and MIU must be '
                             'supplied!')

        if not MIU:
            # Start Flow Control System
            self.flow = LGR.FlowControlSystem(valve_pin)
            self.flow.start()
            time.sleep(10)  # wait for flow to get started before starting LGR

        # Old handler for backwards compatability
        old_path_template = os.path.join(DATAPATH, 'lgr', 'lgr_%Y_%m_%d.dat')
        old_handler = ResponseHandler(old_path_template, LGR.variables,
                                      LGR.filter_response)

        super().__init__(LGR.name, serial_port(port), baudrate=LGR.baudrate,
                         handlers=[LGR.handler, old_handler],
                         filter_response=LGR.filter_response)

        self.valve_pin = valve_pin
        self.MIU = MIU

    def get_data(self) -> Iterator[dict]:
        print('Reading LGR')  # FIXME
        while True:
            timestamp = dt.utcnow()
            response = self.connection.read_until(self.eol_delimiter)

            try:
                response = response.decode('utf-8')
            except UnicodeDecodeError:
                continue

            for handler in self.handlers:
                record = handler.format_response(response, self.logger)
                ID = record['MIU_DESC'] if self.MIU else self.flow.ID
                data = Data(handler.path_template, {'timestamp': timestamp,
                                                    'ID': ID,
                                                    **record})
                yield data


class Magee_AE33(SerialDevice):
    name = 'magee_ae33'

    variables = [
      {'name': 'serial_number', 'save': False},
      {'name': 'inst_date',     'save': False},
      {'name': 'inst_time',     'save': False},
      {'name': 'bc1_ngm3',      'save': True},
      {'name': 'bc2_ngm3',      'save': True},
      {'name': 'bc3_ngm3',      'save': True},
      {'name': 'bc4_ngm3',      'save': True},
      {'name': 'bc5_ngm3',      'save': True},
      {'name': 'bc6_ngm3',      'save': True},
      {'name': 'bc7_ngm3',      'save': True},
      {'name': 'flow_lpm',      'save': True},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False},
      {'name': 'diagnostic',    'save': False}
    ]

    def __init__(self, port):
        super().__init__(Magee_AE33.name, serial_port(port),
                         variables=Magee_AE33.variables)


class MetOne_ES642(SerialDevice):
    name = 'metone_es642'

    variables = [
      {'name': 'pm25_mgm3', 'save': True},
      {'name': 'flow_lpm',  'save': True},
      {'name': 't_c',       'save': True},
      {'name': 'rh_pct',    'save': True},
      {'name': 'pres_hpa',  'save': True},
      {'name': 'status',    'save': True},
      {'name': 'checksum',  'save': True}
    ]

    def __init__(self, port):
        super().__init__(MetOne_ES642.name, serial_port(port),
                         variables=MetOne_ES642.variables)


class Teledyne_T200(SerialDevice):
    name = 'teledyne_t200'

    baudrate = 115200
    delimiter = '\n'
    poll_command = 't list all\r\n'
    poll_interval = 2
    poll_type = 'batch'
    variable_filter = '(?<==)[^\\s]+'

    variables = [
        {'name': 'no_and_nox_ppb',  'save': False},
        {'name': 'range_ppb',       'save': False},
        {'name': 'range1_ppb',      'save': False},
        {'name': 'range2_ppb',      'save': False},
        {'name': 'range3_ppb',      'save': False},
        {'name': 'nox_std_ppb',     'save': True},
        {'name': 'flow_ccm',        'save': True},
        {'name': 'o3_flow_ccm',     'save': True},
        {'name': 'pmt_mv',          'save': True},
        {'name': 'pmt_norm_mv',     'save': True},
        {'name': 'azero_mv',        'save': True},
        {'name': 'hvps_v',          'save': True},
        {'name': 'rcell_t_c',       'save': True},
        {'name': 'box_t_c',         'save': True},
        {'name': 'pmt_t_c',         'save': True},
        {'name': 'moly_t_c',        'save': True},
        {'name': 'rcel_pres_inhga', 'save': True},
        {'name': 'samp_pres_inhga', 'save': True},
        {'name': 'nox_slope',       'save': True},
        {'name': 'nox_offset_mv',   'save': True},
        {'name': 'no_slope',        'save': True},
        {'name': 'no_offset_mv',    'save': True},
        {'name': 'no2_ppb',         'save': True},
        {'name': 'nox_ppb',         'save': True},
        {'name': 'no_ppb',          'save': True},
        {'name': 'test_mv',         'save': True},
        {'name': 'xin1_v',          'save': False},
        {'name': 'xin2_v',          'save': False},
        {'name': 'xin3_v',          'save': False},
        {'name': 'xin4_v',          'save': False},
        {'name': 'xin5_v',          'save': False},
        {'name': 'xin6_v',          'save': False},
        {'name': 'xin7_v',          'save': False},
        {'name': 'xin8_v',          'save': False},
        {'name': 'inst_time',       'save': False}
    ]

    def __init__(self, port):
        super().__init__(Teledyne_T200.name, serial_port(port),
                         variables=Teledyne_T200.variables,
                         baudrate=Teledyne_T200.baudrate,
                         delimiter=Teledyne_T200.delimiter,
                         poll_command=Teledyne_T200.poll_command,
                         poll_interval=Teledyne_T200.poll_interval,
                         poll_type=Teledyne_T200.poll_type,
                         variable_filters=Teledyne_T200.variable_filter)


class Teledyne_T300(SerialDevice):
    name = 'teledyne_t300'

    baudrate = 115200
    delimiter = '\n'
    poll_command = 't list all\r\n'
    poll_interval = 2
    poll_type = 'batch'
    variable_filter = '(?<==)[^\\s]+'

    variables = [
      {'name': 'range_ppb',       'save': False},
      {'name': 'range1_ppb',      'save': False},
      {'name': 'range2_ppb',      'save': False},
      {'name': 'co_std_ppb',      'save': True},
      {'name': 'co_meas_mv',      'save': True},
      {'name': 'co_ref_mv',       'save': True},
      {'name': 'mr_ratio',        'save': True},
      {'name': 'samp_pres_inhga', 'save': True},
      {'name': 'flow_ccm',        'save': True},
      {'name': 'samp_t_c',        'save': True},
      {'name': 'bench_t_c',       'save': True},
      {'name': 'wheel_t_c',       'save': True},
      {'name': 'box_t_c',         'save': True},
      {'name': 'pht_drive_mv',    'save': True},
      {'name': 'slope',           'save': True},
      {'name': 'slope1',          'save': True},
      {'name': 'slope2',          'save': True},
      {'name': 'offset',          'save': True},
      {'name': 'offset1',         'save': True},
      {'name': 'offset2',         'save': True},
      {'name': 'co_ppb',          'save': True},
      {'name': 'test_mv',         'save': True},
      {'name': 'xin1_v',          'save': False},
      {'name': 'xin2_v',          'save': False},
      {'name': 'xin3_v',          'save': False},
      {'name': 'xin4_v',          'save': False},
      {'name': 'xin5_v',          'save': False},
      {'name': 'xin6_v',          'save': False},
      {'name': 'xin7_v',          'save': False},
      {'name': 'xin8_v',          'save': False},
      {'name': 'inst_time',       'save': False}
    ]

    def __init__(self, port):
        super().__init__(Teledyne_T300.name, serial_port(port),
                         variables=Teledyne_T300.variables,
                         baudrate=Teledyne_T300.baudrate,
                         delimiter=Teledyne_T300.delimiter,
                         poll_command=Teledyne_T300.poll_command,
                         poll_interval=Teledyne_T300.poll_interval,
                         poll_type=Teledyne_T300.poll_type,
                         variable_filters=Teledyne_T300.variable_filter)


class Teledyne_T400(SerialDevice):
    name = 'teledyne_t400'

    baudrate = 115200
    delimiter = '\n'
    poll_command = 't list all\r\n'
    poll_interval = 2
    poll_type = 'batch'
    variable_filter = '(?<==)[^\\s]+'

    variables = [
      {'name': 'range_ppb',       'save': False},
      {'name': 'range1_ppb',      'save': False},
      {'name': 'range2_ppb',      'save': False},
      {'name': 'o3_std_ppb',      'save': True},
      {'name': 'o3_meas_mv',      'save': True},
      {'name': 'o3_ref_mv',       'save': True},
      {'name': 'o3_gen_mv',       'save': True},
      {'name': 'o3_drive_mv',     'save': True},
      {'name': 'photo_power_mv',  'save': True},
      {'name': 'samp_pres_inhga', 'save': True},
      {'name': 'flow_ccm',        'save': True},
      {'name': 'samp_t_c',        'save': True},
      {'name': 'photo_lamp_t_c',  'save': True},
      {'name': 'o3_scrub_t_c',    'save': True},
      {'name': 'o3_gen_t_c',      'save': True},
      {'name': 'box_t_c',         'save': True},
      {'name': 'slope',           'save': True},
      {'name': 'offset',          'save': True},
      {'name': 'o3_ppb',          'save': True},
      {'name': 'test_mv',         'save': True},
      {'name': 'xin1_v',          'save': False},
      {'name': 'xin2_v',          'save': False},
      {'name': 'xin3_v',          'save': False},
      {'name': 'xin4_v',          'save': False},
      {'name': 'xin5_v',          'save': False},
      {'name': 'xin6_v',          'save': False},
      {'name': 'xin7_v',          'save': False},
      {'name': 'xin8_v',          'save': False},
      {'name': 'inst_time',       'save': False}
    ]

    def __init__(self, port):
        super().__init__(Teledyne_T400.name, serial_port(port),
                         variables=Teledyne_T400.variables,
                         baudrate=Teledyne_T400.baudrate,
                         delimiter=Teledyne_T400.delimiter,
                         poll_command=Teledyne_T400.poll_command,
                         poll_interval=Teledyne_T400.poll_interval,
                         poll_type=Teledyne_T400.poll_type,
                         variable_filters=Teledyne_T400.variable_filter)


class Teledyne_T500u(SerialDevice):
    name = 'teledyne_t500u'

    baudrate = 115200
    delimiter = '\n'
    poll_command = 't list all\r\n'
    poll_interval = 2
    poll_type = 'batch'
    variable_filter = '(?<==)[^\\s]+'

    variables = [
      {'name': 'range_ppb',       'save': False},
      {'name': 'range1_ppb',      'save': False},
      {'name': 'range2_ppb',      'save': False},
      {'name': 'phase_t_c',       'save': True},
      {'name': 'bench_phase_s',   'save': True},
      {'name': 'meas_l_mm',       'save': True},
      {'name': 'aref_l_mm',       'save': True},
      {'name': 'samp_pres_inhga', 'save': True},
      {'name': 'samp_temp_c',     'save': True},
      {'name': 'bench_t_c',       'save': True},
      {'name': 'box_t_c',         'save': True},
      {'name': 'no2_slope',       'save': True},
      {'name': 'no2_offset_mv',   'save': True},
      {'name': 'no2_ppb',         'save': True},
      {'name': 'bench_no2_s',     'save': False},
      {'name': 'no2_std_ppb',     'save': True},
      {'name': 'mf_t_c',          'save': True},
      {'name': 'ics_t_c',         'save': False},
      {'name': 'sig_mv',          'save': False},
      {'name': 'sin',             'save': False},
      {'name': 'sin_1',           'save': False},
      {'name': 'cos_1',           'save': False},
      {'name': 'sin_2',           'save': False},
      {'name': 'cos_2',           'save': False},
      {'name': 'sin_ovp',         'save': False},
      {'name': 'cos_ovp',         'save': False},
      {'name': 'accum',           'save': False},
      {'name': 'test_mv',         'save': True},
      {'name': 'xin1_v',          'save': False},
      {'name': 'xin2_v',          'save': False},
      {'name': 'xin3_v',          'save': False},
      {'name': 'xin4_v',          'save': False},
      {'name': 'xin5_v',          'save': False},
      {'name': 'xin6_v',          'save': False},
      {'name': 'xin7_v',          'save': False},
      {'name': 'xin8_v',          'save': False},
      {'name': 'time_of_day',     'save': False}
    ]

    def __init__(self, port):
        super().__init__(Teledyne_T500u.name, serial_port(port),
                         variables=Teledyne_T500u.variables,
                         baudrate=Teledyne_T500u.baudrate,
                         delimiter=Teledyne_T500u.delimiter,
                         poll_command=Teledyne_T500u.poll_command,
                         poll_interval=Teledyne_T500u.poll_interval,
                         poll_type=Teledyne_T500u.poll_type,
                         variable_filters=Teledyne_T500u.variable_filter)


class TSI_VWCPC(SerialDevice):
    # TODO docstr
    """https://tsi.com/products/particle-counters-and-detectors/condensation-particle-counters/versatile-water-based-condensation-particle-counter-3789/
    """

    name = 'magic_200p',

    baudrate = 115200
    eol_delimiter = '\r'
    init_command = 'SL,1\r\nSM,3,10\r\n'

    variables = [
      {'name': 'line_type',              'save': False},
      {'name': 'inst_date',              'save': False},
      {'name': 'inst_time',              'save': False},
      {'name': 'status_flag',            'save': True},
      {'name': 'particle_count_nccm',    'save': True},
      {'name': 'accum_particle_count_n', 'save': False},
      {'name': 'unknown',                'save': False},
      {'name': 'unknown',                'save': False},
      {'name': 'pulse_height_mv',        'save': False},
      {'name': 'pulse_height_sd_mv',     'save': False},
      {'name': 'unknown',                'save': False},
      {'name': 'unknown',                'save': False},
      {'name': 'unknown',                'save': False},
      {'name': 'unknown',                'save': False},
      {'name': 'status_record_id',       'save': False},
      {'name': 'conditioner_t_c',        'save': True},
      {'name': 'initiator_t_c',          'save': True},
      {'name': 'optic_t_c',              'save': True},
      {'name': 'water_seperator_t_c',    'save': False},
      {'name': 'case_t_c',               'save': False}
    ]

    def __init__(self, port):
        super().__init__(TSI_VWCPC.name, serial_port(port),
                         baudrate=TSI_VWCPC.baudrate,
                         variables=TSI_VWCPC.variables,
                         eol_delimiter=TSI_VWCPC.eol_delimiter,
                         init_command=TSI_VWCPC.init_command)


class Vaisala_WXT536(SerialDevice):
    name = 'vaisala_wxt536'

    baudrate = 19200
    init_command = '0XU,C=2,I=0,M=P\r\n'
    poll_command = '0R0\r\n'
    poll_interval = 1
    poll_type = 'line'

    variables = [
      {'name': 'wind_dir_deg', 'save': True},
      {'name': 'wind_spd_ms',  'save': True},
      {'name': 't_c',          'save': True},
      {'name': 'rh_pct',       'save': True},
      {'name': 'p_hpa',        'save': True},
      {'name': 'rain_mm',      'save': True},
      {'name': 'heater_t_c',   'save': True},
      {'name': 'heater_v',     'save': False}
    ]

    def __init__(self, port):
        super().__init__(Vaisala_WXT536.name, serial_port(port),
                         variables=Vaisala_WXT536.variables,
                         baudrate=Vaisala_WXT536.baudrate,
                         init_command=Vaisala_WXT536.init_command,
                         poll_command=Vaisala_WXT536.poll_command,
                         poll_interval=Vaisala_WXT536.poll_interval,
                         poll_type=Vaisala_WXT536.poll_type)
