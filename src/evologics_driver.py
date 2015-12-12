#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" Evologics modem driver

S2C product family is supported by this driver, both "normal" and "USBL" versions, both serial and tcp/ip connection protocols.

"""
# TODO: it would be nice to implement some better structure for buffering inbound and outbound data. Right now it is dangerous to send commands to modem while awaiting other independent commands from it.
from __future__ import division

__author__ = 'nick'

import serial
import socket
import time
from collections import deque

# default configs
DEFAULT_SERIAL = {
    'port': '/dev/ttyUSB0',
    'baudrate': 19200,
    'parity': serial.PARITY_NONE,
    'stopbits': serial.STOPBITS_ONE,
    'timeout': 1,
    'bytesize': serial.EIGHTBITS
}

DEFAULT_TCP = {
    'ip': '192.168.0.212',        # Device IP (192.168.0.212 is the USBL)
    'port': 9200,           # Port number
    'type': 'lr',           # 'lr': lister-respawn, 'ls': listen-single, 'c': client
}

# define the length of incoming data messages (in tokens)
DATA_MSG_LENGTHS = {
    'IM': 64,
    'IMS': 64,
    'BURST': 1024,
    'RECV': 10,
    'RECVIM': 10,
    'RECVIMS': 10,
    'RECVPBM': 9
}

DEFAULT_CONFIG = {
    'connection_type': "SERIAL",  # "TCP/IP" or "SERIAL"
    'read_rate': 10,  # Hz
    'status_rate': 2,  # Hz
    'source_level': 3,  # From 0 (max) to 3 (min)
    'serial_config': DEFAULT_SERIAL,
    'tcp_config': DEFAULT_TCP
}

# errors and warns
WARN_ACK_BUSY = -1  # new ack requested before previous ack was received, overwriting the ack
OK = 0
ERROR_DATA_TOO_LONG = 1
ERROR_MODEM_BUSY = 2  # modem is currently executing another command

ERROR_WRONG_TYPE = -1  # bad response of the modem (command-response mismatch)
ERROR_TOO_MANY_LINES = -2  # more lines than expected
ERROR_NO_LINES = -3  # no lines read - connection problem?

# modem commands
SET = {
    'source_level':     'AT!L',
    'system_time':      'AT!UT',
    'positioning_data_output': 'AT@ZU'
}

GET = {
    'battery_voltage':  'AT?BV',
    'system_time':      'AT?UT',
    'propagation_time': 'AT?T'
}

ACK_STRING = {
    True: 'ack',
    False: 'noack'
}

ACK_FLAG = {
    True: 'imack',
    False: ''
}
class ModemDriver(object):
    """This is an abstract class and should never be used directly. Use the children of this class. It contains all
    the modem communication methods that are independent of the interface (serial or tcp/ip).
    """
    def __init__(self, **kwargs):
        """
        :param kwargs: expected a dictionary with a format as DEFAULT_CONFIG
        """
        # not used yet
        self.write_buffer = deque()
        self.read_buffer = deque()

        self.flag_busy = False
        self.last_ack_type = ''
        self.set_modem_params(kwargs.get('source_level', DEFAULT_CONFIG['source_level']))

    def set_modem_params(self, source_level):
        #self.sync_modem_time()
        self.set_source_level(source_level)
        self.set_positioning_data(1)

    # def sync_modem_time(self, time):
    #     """
    #     Send command to set the device time as the rostime. This command should be used only during initialisation of the
    #     modem. (because it disrupts reading from device)
    #     """
    #     # time = rospy.Time.now().to_sec()
    #     serMsg = 'AT!UT{0}'.format(time)
    #     self.send_command(serMsg)
    #     time.sleep(0.5)
    #     self.read_device_msg()

    def set_source_level(self, source_level):
        """
        Send command to set the output acoustic power. This command should be used only during initialisation of the
        modem. (because it disrupts reading from device)
        :param source_level: it can be from 3 (minimum) to 0 (maximum)
        """
        cmd = '{0}{1}'.format(SET['source_level'], source_level)
        self.add_to_write_buffer(cmd, left=False)
        self.send_line()
        time.sleep(0.5)
        self.read_device_to_buffer()
        if len(self.read_buffer) > 0:
            line = self.read_buffer.pop()
        else:
            line = 'Modem not connected!'
        return line

    def set_positioning_data(self, n):
        cmd = '{0}{1}'.format(SET['positioning_data_output'], n)
        self.send_command(cmd)
        time.sleep(0.5)
        return self.read_device_to_buffer()

    def add_im_to_buffer(self, destination_address, ack, data):
        """
        Sends instant message to a specific acoustic address. If ack is high and currently awaited ack has not arrived
        yet it will be no longer tracked (ack flag is overwritten). If modem is busy sending other message the request
        will be discarded and an ERROR_MODEM_BUSY returned.

        Instant messages are sent as soon as possible and can be sent in parallel with burst messages. The bitrate is
        fixed at 976 bps.

        :param destination_address:
        :param ack: should the device await the ack signal?
        :param data: at most 64 bytes
        :return: error codes
        """
        cmd = ''
        if len(data) > DATA_MSG_LENGTHS['IM']:
            return ERROR_DATA_TOO_LONG, cmd

        ack_str = ACK_STRING[ack]
        cmd = 'AT*SENDIM,{0},{1},{2},{3}'.format(len(data), destination_address, ack_str, data)

        self.add_to_write_buffer(cmd, ack=ACK_FLAG[ack])

        return OK, cmd

    # TODO: finish implementation, add safe time sync
    # def send_ims(self, destination_address, timestamp, data):
    #     if len(data) > DATA_MSG_LENGTHS['IMS']:
    #         return ERROR_DATA_TOO_LONG
    #
    #     cmd = 'AT*SENDIMS,{0},{1},{2},{3}'.format(len(data), destination_address, timestamp, data)
    #
    #     ack_str = ''
    #
    #     if self.flag_busy:
    #         return ERROR_MODEM_BUSY
    #
    #     self.send_command(cmd)
    #     if self.last_ack_type != '':
    #         self.last_ack_type = ack_str
    #         return WARN_ACK_BUSY
    #
    #     self.last_ack_type = ack_str
    #     return OK

    def add_burst_to_buffer(self, destination, data):
        """
        Sends burst of data to a specific address. Generally, bitrate of burst messages is higher - it is determined
        automatically based on the quality of the acoustic link (up to 13kbps).

        :param destination:
        :param data: at most 1024 bytes
        :return: error codes
        """
        cmd = ''
        if len(data) > DATA_MSG_LENGTHS['BURST']:
            return ERROR_DATA_TOO_LONG

        cmd = 'AT*SEND,{0},{1},{2}'.format(len(data), destination, data)
        self.add_to_write_buffer(cmd, ack='burstack')

        return OK, cmd

    def read_battery_level(self):
        """
        Checks the voltage level of the power source. This function is currently dangerous as it reads all lines
        available from the modem connection but expects only one (voltage response from the modem). If there are more
        lines available the voltage read fails and all of the lines are passed as the second return value.

        :return: voltage level of the power source of the modem (-1 or -2 if failed), extra lines that can be passed to
            some parser
        """
        if len(self.read_buffer) > 0:
            return ''
        cmd = GET['battery_voltage']
        self.add_to_write_buffer(cmd, left=False)
        return self.send_line()

    def read_propagation_time(self):
        """

        """
        if len(self.read_buffer) > 0:
            return ''
        cmd = GET['propagation_time']
        self.add_to_write_buffer(cmd, left=False)
        return self.send_line()

    def add_to_write_buffer(self, cmd, left=True, ack=''):
        if left:
            self.write_buffer.appendleft((cmd, ack))
        else:
            self.write_buffer.append((cmd, ack))

    def send_line(self):
        if self.flag_busy:
            return ERROR_MODEM_BUSY, ''

        if len(self.write_buffer) > 0:
            cmd, ack = self.write_buffer.pop()
            # print cmd, 'sent!!'
            self.send_command(cmd)
            if self.last_ack_type != '' and ack != '':
                self.last_ack_type = ack
                return WARN_ACK_BUSY, cmd
            elif self.last_ack_type == '':
                self.last_ack_type = ack

            return OK, cmd
        # TODO: add different return for when buffer empty?
        return OK, ''

    # Dummy function shadowed by the children of this class
    def read_device_to_buffer(self):
        raise ValueError("ModemDriver class is an abstract class. It should not be used directly")

    # Dummy function shadowed by the children of this class
    def send_command(self, cmd):
        raise ValueError("ModemDriver class is an abstract class. It should not be used directly")

    # Dummy function shadowed by the children of this class
    def close(self):
        raise ValueError("ModemDriver class is an abstract class. It should not be used directly")


class SerialModemDriver(ModemDriver):
    """
    This is a child class of the ModemDriver. Use this if you want to use serial to connect to the modem.
    """
    def __init__(self, **kwargs):
        serial_config = kwargs.get('serial_config', DEFAULT_SERIAL)
        self.ser = serial.Serial(**serial_config)
        self.ser.flushInput()

        super(SerialModemDriver, self).__init__()
        # ModemDriver.__init__(self, **kwargs)

    def send_command(self, cmd):
        """
        Send a string to the device trough the selected connection
        :param cmd: a pre-formatted command
        """
        self.flag_busy = True
        self.ser.write(cmd+'\r')

    def read_device_to_buffer(self):
        """
        This method reads incoming data from the port selected.
        It also check if the string received is complete or if there are more than 1 messages packed together
        :return: a list of strings containing the info from the device
        """
        lines = []
        line = self.ser.readline()
        tokens = line.split(',')
        if tokens[0] in DATA_MSG_LENGTHS:
            tokens[DATA_MSG_LENGTHS[tokens[0]] -1] = ',incoming '.join(tokens[(DATA_MSG_LENGTHS[tokens[0]] -1):])
            tokens = tokens[:DATA_MSG_LENGTHS[tokens[0]]]
            # Verify if the payload is complete, the data piece has to be equal to the second token + 2 (\r\n)
            len_error = int(tokens[1]) - len(tokens[DATA_MSG_LENGTHS[tokens[0]] -1]) + 2
            if len_error > 0:
                line += self.ser.read(len_error)
            elif len_error < 0:
                # rospy.logerr("%s: The payload is longer then expected" % (self.name))
                return []
        if line != "":
            # append data without "\r\n"
            lines.append(line[:-2])

        if len(line) > 0:
            self.flag_busy = False
            self.send_line()

        for line in lines:
            self.read_buffer.appendleft(line)


class TCPModemDriver(ModemDriver):
    """
    This is a child class of the ModemDriver. Use this if you want to use tcp/ip to connect to the modem.
    """
    def __init__(self, **kwargs):

        tcp_config = kwargs.get('tcp_config', DEFAULT_TCP)
        self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp.connect((tcp_config['ip'], tcp_config['port']))
        self.tcp.setblocking(0)
        # Clear the buffer of old unwanted messages
        try:
            to_discard = self.tcp.recv(4096)
        except socket.error:
            pass

        super(TCPModemDriver, self).__init__()
        # ModemDriver.__init__(self, **kwargs)

    def send_command(self, cmd):
        """
        Send a string to the device through the selected connection
        :param cmd: a pre-formatted command
        """
        self.flag_busy = True
        self.tcp.send(cmd+'\n')

    def read_device_to_buffer(self):
        """
        This method reads incoming data from the port selected.
        It also check if the string received is complete or if there are more than 1 messages packed together
        :return: a list of strings containing the info from the device
        """
        lines = []
        try:
            line = self.tcp.recv(4096)
            if not line[-2:] == '\r\n':
                time.sleep(0.1)
                line += self.tcp.recv(4096)
        except socket.error:
            return []
        # print "Whole line: " , repr(line)
        # There can be more than 1 message, so we need to split
        rows = line.split("\r\n")
        while len(rows) != 0:
            row = rows.pop(0)
            tokens = row.split(',')
            # print tokens
            if tokens[0] in DATA_MSG_LENGTHS:
                tokens[DATA_MSG_LENGTHS[tokens[0]] -1] = ','.join(tokens[(DATA_MSG_LENGTHS[tokens[0]] -1):])
                tokens = tokens[:DATA_MSG_LENGTHS[tokens[0]]]
                # Verify if the payload is complete, the data piece has to be equal to the second token
                len_error = int(tokens[1]) - len(tokens[DATA_MSG_LENGTHS[tokens[0]] -1])
                while len_error > 0:
                    # print "Missing" , len_error , "chars, next one: " , repr(rows[0])
                    tokens[tokens[DATA_MSG_LENGTHS[tokens[0]] - 1]] += "\r\n"
                    tokens[tokens[DATA_MSG_LENGTHS[tokens[0]] - 1]] += rows.pop(0)
                    len_error = int(tokens[1]) - len(tokens[DATA_MSG_LENGTHS[tokens[0]] -1])
                    row = ','.join(tokens)
                if len_error < 0:
                    raise ValueError('The payload is longer then expected')
            if row != "":
                lines.append(row)
                # print "ROW: " , repr(row)

        if len(line) > 0:
            self.flag_busy = False
            self.send_line()

        for line in lines:
            self.read_buffer.appendleft(line)

    def close(self):
        self.tcp.close()
