#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" Evologics modem driver

This is a ROS node which works as a bridge between the software and the Evologics devices.
S2C product family is supported by this driver, both "normal" and "USBL" versions, both serial and tcp/ip connection protocols.

"""
# TODO: add propagation time extract
# TODO: add relative velocity extract

from __future__ import division

__author__ = 'nick'

import argparse
import traceback
import numpy as np  # do I need numpy???
np.set_printoptions(precision=3, suppress=True)
import serial
import socket
import sys
import time

#ROS imports
import rospy
import roslib
roslib.load_manifest('evologics_driver')

from vehicle_interface.msg import AcousticModemStatus, AcousticModemPayload, AcousticModemAck, AcousticModemUSBLANGLES, AcousticModemUSBLLONG
from diagnostic_msgs.msg import KeyValue
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# config
# CONNECTION_TYPE = "TCP/IP"                  # "TCP/IP" or "SERIAL"
CONNECTION_TYPE = "SERIAL"                  # "TCP/IP" or "SERIAL"
DEFAULT_RATE = 10                           # Hz
STATUS_RATE = 2                             # Hz
SOURCE_LEVEL = 3                            # From 0 (max) to 3 (min)

# modem status
MODEM_DISABLED = 0
MODEM_ENABLED = 1

MODEM_STATUS = {
    MODEM_DISABLED: AcousticModemStatus.MODEM_DISABLED,
    MODEM_ENABLED: AcousticModemStatus.MODEM_ENABLED
}

# ros topics
# instant message
TOPIC_IM_IN = 'modem/im/in'
TOPIC_IM_OUT = 'modem/im/out'
TOPIC_IM_ACK = 'modem/im/ack'

# synchronous instant message
TOPIC_IMS_IN = 'modem/ims/in'
TOPIC_IMS_OUT = 'modem/ims/out'
TOPIC_IMS_ACK = 'modem/ims/ack'

# burst message
TOPIC_BURST_IN = 'modem/burst/in'
TOPIC_BURST_OUT = 'modem/burst/out'
TOPIC_BURST_ACK = 'modem/burst/ack'

# modem status
TOPIC_STATUS = 'modem/status'

# USBL data
TOPIC_USBLLLONG = "modem/usbllong"
TOPIC_USBLANGLES = "modem/usblangles"

# modem switch service
SRV_SWITCH = 'modem/switch'


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
DATA_MSGS =  {
    'RECV': 10,
    'RECVIM': 10,
    'RECVIMS': 10,
    'RECVPBM': 9
}


class ModemDriver(object):
    """ModemDriver class represents the ROS interface for the Evologics Acoustic Modems

      This class implements all the required functionality to communicate with the modems. It allows transmission and
      reception of different types of messages as described in the user manual. It also provides localisation
      information for USBL modems and modem status information.
    """

    flag_BUSY = False
    flag_ack_BUSY = False
    last_transmission_type = ""

    def __init__(self, name, driver_rate, **kwargs):
        """ Class initialization method

        This method sets the node parameters: node name, update rate, etc...
        It also initializes publishers and subscribers and opens the connection with the device
        :param name: name of the node
        :param driver_rate: loop frequency
        :param kwargs:
        """
        self.name = name
        self.driver_rate = driver_rate

        self.verbose = kwargs.get('verbose', False)
        # Dictionary containing the message types sent by the device, when called it redirects you to the proper method
        # TODO complete the dictionary for all device inputs
        self.dict_messages_types = {
            'RECV': self.parse_acoustic_bm,
            'DELIVERED': self.parse_delivered,
            'FAILED': self.parse_failed,
            'RECVIM': self.parse_acoustic_im,
            'DELIVEREDIM': self.parse_delivered,
            'FAILEDIM': self.parse_failed,
            'USBLLONG': self.publish_usbllong_msg,
            'USBLANGLES': self.publish_usblangles_msg,
            'OK': self.parse_ok,
            '[*]OK': self.parse_ok
        }

        # timing
        self.dt = 1.0 / self.driver_rate
        self.driver_loop = rospy.Rate(self.driver_rate)
        self.driver_status = rospy.Timer(rospy.Duration(1.0 / STATUS_RATE), self.send_status)

        # initial status
        self.modem_status = MODEM_ENABLED

        # instant message publishers and subscribers
        self.pub_im_in = rospy.Publisher(TOPIC_IM_IN, AcousticModemPayload, tcp_nodelay=True, queue_size=1)
        self.pub_im_ack = rospy.Publisher(TOPIC_IM_ACK, AcousticModemAck, tcp_nodelay=True, queue_size=1)
        self.sub_im_out = rospy.Subscriber(TOPIC_IM_OUT, AcousticModemPayload, self.handle_im_out, tcp_nodelay=True, queue_size=1)

        # synchronous instant message publishers and subscribers
        self.pub_ims_in = rospy.Publisher(TOPIC_IMS_IN, AcousticModemPayload, tcp_nodelay=True, queue_size=1)
        self.pub_ims_ack = rospy.Publisher(TOPIC_IMS_ACK, AcousticModemAck, tcp_nodelay=True, queue_size=1)
        self.sub_ims_out = rospy.Subscriber(TOPIC_IMS_OUT, AcousticModemPayload, self.handle_ims_out, tcp_nodelay=True, queue_size=1)

        # burst message publishers and subscribers
        self.pub_burst_in = rospy.Publisher(TOPIC_BURST_IN, AcousticModemPayload, tcp_nodelay=True, queue_size=1)
        self.pub_burst_ack = rospy.Publisher(TOPIC_BURST_ACK, AcousticModemAck, tcp_nodelay=True, queue_size=1)
        self.sub_burst_out = rospy.Subscriber(TOPIC_BURST_OUT, AcousticModemPayload, self.handle_burst_out, tcp_nodelay=True, queue_size=1)

        # USBL data publishers
        self.pub_usbllong = rospy.Publisher(TOPIC_USBLLLONG, AcousticModemUSBLLONG, tcp_nodelay=True, queue_size=1)
        self.pub_usblangles = rospy.Publisher(TOPIC_USBLANGLES, AcousticModemUSBLANGLES, tcp_nodelay=True, queue_size=1)

        # status publisher
        self.pub_status = rospy.Publisher(TOPIC_STATUS, AcousticModemStatus, tcp_nodelay=True, queue_size=1)

        # services
        self.s_switch = rospy.Service(SRV_SWITCH, BooleanService, self.srv_switch)

        # optional info
        if self.verbose:
            t_pri = rospy.Timer(rospy.Duration(0.5), self.print_info)

        # create serial port connection TODO: Read parameters from config/launch file, also check the timeout option
        if CONNECTION_TYPE == "SERIAL":
            self.ser = serial.Serial(**DEFAULT_SERIAL)
            #self.ser.open()
            self.ser.flushInput()
        elif CONNECTION_TYPE == "TCP/IP":
            try:
                self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp.connect((DEFAULT_TCP['ip'], DEFAULT_TCP['port']))
                self.tcp.setblocking(0)
                # Clear the buffer of old unwanted messages
                try:
                    to_discard = self.tcp.recv(4096)
                except socket.error:
                    pass
            except socket.error:
                rospy.logerr('%s: There was an error while opening the socket' % self.name)
                sys.exit()
        else:
            pass

        self.__set_modem_params()
        rospy.loginfo('%s: Modem connected' % self.name)

    def __set_modem_params(self):
        #self.sync_modem_time()
        self.set_source_level(SOURCE_LEVEL)

    def sync_modem_time(self):
        """
        Send command to set the device time as the rostime
        """
        serMsg = 'AT!UT{0}'.format(rospy.Time.now().to_sec())
        self.__send_command(serMsg)
        time.sleep(0.5)
        self.read_device_msg()


    # safety switch service
    def srv_switch(self, req):
        """This method handles the switch service.

        This will enable/disable the modem driver.
        """
        if req.request is True:
            # enable the low-level controller
            self.modem_status = MODEM_ENABLED
            return BooleanServiceResponse(True)
        else:
            self.modem_status = MODEM_DISABLED
            return BooleanServiceResponse(False)

    # status publisher
    def send_status(self, event=None):
        ms = AcousticModemStatus()
        ms.header.stamp = rospy.Time.now()
        ms.status = MODEM_STATUS[self.modem_status]

        self.pub_status.publish(ms)

    def set_source_level(self, source_level):
        """
        Send command to set the output acoustic power
        :param source_level: it can be from 3 (minimum) to 0 (maximum)
        """
        serMsg = 'AT!L{0}'.format(source_level)
        self.__send_command(serMsg)
        time.sleep(0.5)
        self.read_device_msg()

    def __send_command(self, cmd):
        """
        Send a string to the device trough the selected connection
        :param cmd: a pre-formatted command
        :return: True if the command is correctly sent, False if the device is busy (waiting for an acknowledge)
        """
        rospy.loginfo("%s: Sending Command to modem: %s" %(self.name,cmd.strip()))
        if self.flag_BUSY:
            rospy.logerr("%s: Device busy" % self.name)
            return False
        if self.flag_ack_BUSY:
            rospy.logwarn("%s: Still waiting for ack")
        self.flag_BUSY = True
        if self.modem_status:
            if CONNECTION_TYPE == "SERIAL":
                cmd += '\r'
                self.ser.write(cmd)
            if CONNECTION_TYPE == "TCP/IP":
                cmd += '\n'
                self.tcp.send(cmd)
        else:
            rospy.logdebug('%s: [cmd]: %s' % (self.name, cmd.strip()))
        return True

    def handle_im_out(self, msg):
        """
        ROS callback
        Send an instant message to the device
        :param msg: ROS message, type: AcousticModemPayload
        """
        ack = 'noack'

        if msg.ack is True:
            ack = 'ack'

        serMsg = 'AT*SENDIM,{0},{1},{2},{3}'.format(
            len(msg.payload), msg.address, ack, msg.payload
        )
        if self.__send_command(serMsg) and (ack == "ack"):
            self.flag_ack_BUSY = True
            self.last_transmission_type = "imack"

    def handle_ims_out(self, msg):
        # Publish to serial port
        # To be done
        pass

    def handle_burst_out(self, msg):
        """
        ROS callback
        Send an burst message to the device
        :param msg: ROS message, type: AcousticModemPayload
        """
        serMsg = 'AT*SEND,{0},{1},{2}'.format(
            len(msg.payload), msg.address, msg.payload
        )
        if self.__send_command(serMsg):
            self.flag_ack_BUSY = True
            self.last_transmission_type = "burstack"
        pass


    # Parsing methods
    def parse_acoustic_bm(self, tokens):
        """
        This method parse an incoming burst message and publish its content on a dedicated topic
        :param tokens: list of message fields
        """
        msg = AcousticModemPayload()
        msg.address = int(tokens[2])
        msg.payload = ','.join(tokens[9:])

        duration = int(tokens[7])  # in microseconds
        rel_speed = float(tokens[8])  # in meters per second
        info = {
            'duration': duration,
            'relative_speed': rel_speed
        }
        msg.info = [KeyValue(key, value) for key, value in info.items()]

        rospy.loginfo('%s: Received IM from node %s, payload: %s, info: %s' %(self.name, msg.address, repr(msg.payload), repr(msg.info)))
        self.pub_burst_in.publish(msg)

    def parse_acoustic_im(self, tokens):
        """
        This method parses an incoming instant message and publish its content on a dedicated topic
        :param tokens: list of message fields
        """
        msg = AcousticModemPayload()
        msg.address = int(tokens[2])
        msg.payload = ','.join(tokens[9:])

        duration = int(tokens[5])  # in microseconds
        rel_speed = float(tokens[8])  # in meters per second
        info = {
            'duration': duration,
            'relative_speed': rel_speed
        }
        msg.info = [KeyValue(key, value) for key, value in info.items()]

        rospy.loginfo('%s: Received IM from node %s, payload: %s, info: %s' %(self.name, msg.address, repr(msg.payload), repr(msg.info)))

        self.pub_im_in.publish(msg)

    def publish_usbllong_msg(self, tokens):
        """
        method which will generate a ros message from USBL data
        :param tokens: list of message fields
        """
        if len(tokens) != 17:
            rospy.logerr("%s: USBLLONG message fields count doesn't match" % self.name)
            return
        usblmsg = AcousticModemUSBLLONG()
        usblmsg.measurement_time = float(tokens[2])
        usblmsg.remote_address = int(tokens[3])
        usblmsg.X = float(tokens[4])
        usblmsg.Y = float(tokens[5])
        usblmsg.Z = float(tokens[6])
        usblmsg.E = float(tokens[7])
        usblmsg.N = float(tokens[8])
        usblmsg.U = float(tokens[9])
        usblmsg.roll = float(tokens[10])    # RPY of local device
        usblmsg.pitch = float(tokens[11])
        usblmsg.yaw = float(tokens[12])
        usblmsg.propagation_time = float(tokens[13])
        usblmsg.accuracy = float(tokens[-1])

        rospy.loginfo('%s: Received USBLLONG data' %(self.name))
        self.pub_usbllong.publish(usblmsg)

    def publish_usblangles_msg(self, tokens):
        """
        method which will generate a ros message from USBL data
        :param tokens: list of message fields
        """
        if len(tokens) != 14:
            rospy.logerr("%s: USBLANGLES message fields count doesn't match" % self.name)
            return
        usblmsg = AcousticModemUSBLANGLES()
        usblmsg.measurement_time = float(tokens[2])
        usblmsg.remote_address = int(tokens[3])
        usblmsg.lbearing = float(tokens[4])
        usblmsg.lelevation = float(tokens[5])
        usblmsg.bearing = float(tokens[6])
        usblmsg.elevation = float(tokens[7])
        usblmsg.roll = float(tokens[8])    # RPY of local device
        usblmsg.pitch = float(tokens[9])
        usblmsg.yaw = float(tokens[10])
        usblmsg.accuracy = float(tokens[13])

        rospy.loginfo('%s: Received USBLANGLES data' %(self.name))
        self.pub_usblangles.publish(usblmsg)

    def parse_delivered(self, tokens):
        rospy.loginfo("%s: Message delivered to node %s" %(self.name,tokens[1]))
        if self.flag_ack_BUSY:
            if self.last_transmission_type == "imack":
                self.pub_im_ack.publish(AcousticModemAck(ack=True))
            elif self.last_transmission_type == "burstack":
                self.pub_burst_ack.publish(AcousticModemAck(ack=True))
        self.flag_ack_BUSY = False

    def parse_failed(self, tokens):
        rospy.logwarn("%s: Message delivering failed to node %s" %(self.name,tokens[1]))
        if self.flag_ack_BUSY:
            if self.last_transmission_type == "imack":
                self.pub_im_ack.publish(AcousticModemAck(ack=False))
            elif self.last_transmission_type == "burstack":
                self.pub_burst_ack.publish(AcousticModemAck(ack=False))
        self.flag_ack_BUSY = False

    def parse_ok(self,tokens):
        # TODO Do something useful when OK message is received from device
        pass

    def read_device_msg(self):
        """
        This method reads incoming data from the port selected.
        It also check if the string received is complete or if there are more than 1 messages packed together
        :return: a list of strings containing the info from the device
        """
        lines = []
        if CONNECTION_TYPE == "SERIAL":
            line = self.ser.readline()
            tokens = line.split(',')
            if tokens[0] in DATA_MSGS:
                tokens[DATA_MSGS[tokens[0]] -1] = ','.join(tokens[(DATA_MSGS[tokens[0]] -1):])
                tokens = tokens[:DATA_MSGS[tokens[0]]]
                # Verify if the payload is complete, the data piece has to be equal to the second token + 2 (\r\n)
                len_error = int(tokens[1]) - len(tokens[DATA_MSGS[tokens[0]] -1]) + 2
                if len_error > 0:
                    line += self.ser.read(len_error)
                elif len_error < 0:
                    rospy.logerr("%s: The payload is longer then expected" % (self.name))
                    return []
            if line != "":
                # append data without "\r\n"
                lines.append(line[:-2])
        elif CONNECTION_TYPE == "TCP/IP":
            try:
                line = self.tcp.recv(4096)
                if not line[-2:] == '\r\n':
                    time.sleep(0.1)
                    line += self.tcp.recv(4096)

            except socket.error:
                return []
            #print "Whole line: " , repr(line)
            # There can be more than 1 message, so we need to split
            rows = line.split("\r\n")
            while len(rows) != 0:
                row = rows.pop(0)
                tokens = row.split(',')
                #print tokens
                if tokens[0] in DATA_MSGS:
                    tokens[DATA_MSGS[tokens[0]] -1] = ','.join(tokens[(DATA_MSGS[tokens[0]] -1):])
                    tokens = tokens[:DATA_MSGS[tokens[0]]]
                    # Verify if the payload is complete, the data piece has to be equal to the second token
                    len_error = int(tokens[1]) - len(tokens[DATA_MSGS[tokens[0]] -1])
                    while len_error > 0:
                        #print "Missing" , len_error , "chars, next one: " , repr(rows[0])
                        tokens[tokens[DATA_MSGS[tokens[0]] -1]] += "\r\n"
                        tokens[tokens[DATA_MSGS[tokens[0]] -1]] += rows.pop(0)
                        len_error = int(tokens[1]) - len(tokens[DATA_MSGS[tokens[0]] -1])
                        row = ','.join(tokens)
                    if len_error < 0:
                        rospy.logerr("%s: The payload is longer then expected" % (self.name))
                        return []
                if row != "":
                    lines.append(row)
                    #print "ROW: " , repr(row)
        else:
            pass
        for line in lines:
            rospy.loginfo('%s: Received line: %s' %(self.name,repr(line)))
            self.flag_BUSY = False

        return lines

    def loop(self):
        """
        Function called in the main loop, just read what is coming on the port and parse it using a dictionary
        The frequency is set by the parameter DEFAULT_RATE
        """
        lines = self.read_device_msg()

        for line in lines:
            #line = line.strip("\r\n")
            tokens = line.split(',')
            first_token = tokens[0]  # command
            if first_token in self.dict_messages_types:
                self.dict_messages_types[first_token](tokens)
        pass

    def run(self):
        # driver loop
        while not rospy.is_shutdown():
            self.loop()
            try:
                self.driver_loop.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo('%s shutdown requested ...', self.name)

        if CONNECTION_TYPE == "SERIAL":
            self.ser.close()
        elif CONNECTION_TYPE == "TCP/IP":
            self.tcp.close()
        else:
            pass

    # verbose info print
    def print_info(self, event=None):
        print(self)

    def __str__(self):
        return """modem:
          enabled: %s
        """ % (
            self.modem_status
        )


def main():
    rospy.init_node('modem_driver')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # parse args
    #   ros_args[0] is always the program name
    ros_args = rospy.myargv()

    parser = argparse.ArgumentParser(
        description='Modem Driver ROS Node. This node is communicating with the Evologics modem.',
        #epilog='This is part of vehicle_pilot module.'
    )
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')
    args = parser.parse_args(ros_args[1:])

    if args.verbose:
        verbose = True
    else:
        verbose = False

    # load global parameters
    rate = int(rospy.get_param('~modem_rate', DEFAULT_RATE))

    rate = int(np.clip(rate, 1, 100).astype(int))

    # show current settings
    rospy.loginfo('%s modem rate: %s Hz', name, rate)

    # (reduce verbosity) show current config
    # rospy.loginfo('%s pitch enabled: %s', name, config['pitch_enable'])
    # rospy.loginfo('%s adaptive yaw enabled: %s', name, config.get('adaptive_yaw', False))
    # rospy.loginfo('%s fault control enabled: %s', name, config.get('fault_control', False))
    # rospy.loginfo('%s fault speeds enabled: %s', name, config.get('fault_speeds', False))

    # start vehicle control node
    driver = ModemDriver(name, rate, verbose=verbose)

    try:
        driver.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s uncaught exception, dying!\n%s', name, tb)


if __name__ == '__main__':
    main()
