#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" Evologics modem driver

This is a ROS node which works as a bridge between the software and the Evologics devices.
S2C product family is supported by this driver, both "normal" and "USBL" versions, both serial and tcp/ip connection protocols.

"""
# TODO: Test im, burst sending
# TODO: Add documentation

from __future__ import division

__author__ = 'nick'

import argparse
import traceback
import numpy as np  # do I need numpy???
np.set_printoptions(precision=3, suppress=True)
import serial
import time
import sys

#ROS imports
import roslib
roslib.load_manifest('evologics_driver')
import rospy
import evologics_driver as ed

from vehicle_interface.msg import AcousticModemStatus, AcousticModemPayload, AcousticModemAck, AcousticModemUSBLANGLES, AcousticModemUSBLLONG
from diagnostic_msgs.msg import KeyValue
from vehicle_interface.srv import BooleanService, BooleanServiceResponse


# modem status
MODEM_DISABLED = 0
MODEM_ENABLED = 1

OK = 0
ERROR = 1

TIMEOUT_PROP_READ = 5

MODEM_STATUS = {
    MODEM_DISABLED: AcousticModemStatus.MODEM_DISABLED,
    MODEM_ENABLED: AcousticModemStatus.MODEM_ENABLED
}

# ros topics
# instant message
TOPIC_IM_IN = 'im/in'
TOPIC_IM_OUT = 'im/out'
TOPIC_IM_ACK = 'im/ack'

# synchronous instant message
TOPIC_IMS_IN = 'ims/in'
TOPIC_IMS_OUT = 'ims/out'
TOPIC_IMS_ACK = 'ims/ack'

# burst message
TOPIC_BURST_IN = 'burst/in'
TOPIC_BURST_OUT = 'burst/out'
TOPIC_BURST_ACK = 'burst/ack'

# modem status
TOPIC_STATUS = 'status'

# USBL data
TOPIC_USBLLLONG = "usbllong"
TOPIC_USBLANGLES = "usblangles"

# modem switch service
SRV_SWITCH = 'switch'
SRV_BATTERY = 'battery'

# default configs
DEFAULT_SERIAL = {
    'port': '/dev/ttyUSB0',
    'baudrate': 19200,
    'parity': serial.PARITY_NONE,
    'stopbits': serial.STOPBITS_ONE,
    'timeout': 0.1,
    'bytesize': serial.EIGHTBITS
}

DEFAULT_TCP = {
    'ip': '192.168.0.212',        # Device IP (192.168.0.212 is the USBL)
    'port': 9200,           # Port number
    'type': 'lr',           # 'lr': lister-respawn, 'ls': listen-single, 'c': client
}

DEFAULT_CONFIG = {
    'connection_type': "SERIAL",  # "TCP/IP" or "SERIAL"
    'read_rate': 10,  # Hz
    'battery_read_rate': 0.01,  # Hz
    'status_rate': 0.1,  # Hz
    'source_level': 3,  # From 0 (max) to 3 (min)
    'serial_config': DEFAULT_SERIAL,
    'tcp_config': DEFAULT_TCP
}

# define the length of incoming data messages (in tokens)
DATA_MSGS_LENGTHS = {
    'IM': 64,
    'BURST': 1024,
    'RECV': 10,
    'RECVIM': 10,
    'RECVIMS': 10,
    'RECVPBM': 9
}

class ModemNode(object):
    """ModemDriver class represents the ROS interface for the Evologics Acoustic Modems

      This class implements all the required functionality to communicate with the modems. It allows transmission and
      reception of different types of messages as described in the user manual. It also provides localisation
      information for USBL modems and modem status information.
    """
    def __init__(self, name, config, **kwargs):
        """ Class initialization method

        This method sets the node parameters: node name, update rate, etc...
        It also initializes publishers and subscribers and opens the connection with the device
        :param name: name of the node
        :param config: dictionary with the configuration (same structure as DEFAULT_CONFIG)
        :param kwargs: verbose
        """
        self.name = name

        self.sent_im_ack_cnt = 0
        self.sent_burst_cnt = 0
        self.received_cnt = 0
        self.im_ack_cnt = 0
        self.burst_ack_cnt = 0

        if config['connection_type'] == "SERIAL":
            self.modem = ed.SerialModemDriver(serial_config=config['serial_config'])
        elif config['connection_type'] == "TCP/IP":
            self.modem = ed.TCPModemDriver(tcp_config=config['tcp_config'])
        rospy.loginfo('%s: Modem connected' % self.name)
        line = self.modem.set_source_level(config['source_level'])

        rospy.loginfo('%s: Received line: %s' % (self.name, repr(line)))

        self.verbose = kwargs.get('verbose', False)
        # Dictionary containing the message types sent by the device, when called it redirects you to the proper method
        # TODO complete the dictionary for all device inputs
        # TODO: add default
        self.dict_messages_types = {
            'RECV': self.parse_acoustic_bm,
            'DELIVERED': self.parse_delivered,
            'FAILED': self.parse_failed,
            'RECVIM': self.parse_acoustic_im,
            'DELIVEREDIM': self.parse_delivered,
            'FAILEDIM': self.parse_failed,
            'USBLLONG': self.parse_usbllong,
            'USBLANGLES': self.parse_usblangles,
            'OK': self.parse_ok,
            '[*]OK': self.parse_ok
        }

        # timing
        self.read_rate = config['read_rate']
        self.dt_read = 1.0 / self.read_rate
        self.driver_loop = rospy.Rate(self.read_rate)

        # TODO: Timers are not supported on ROS diamondback [Emily boat]
        # self.driver_status = rospy.Timer(rospy.Duration(1.0 / config['status_rate']), self.send_status)
        # self.battery_read = ros   py.Timer(rospy.Duration(1.0 / config['battery_read_rate']), ___)

        self.status_period = 1.0/config['status_rate']
        self.last_status_t = 0

        self.battery_read_period = 1/config['battery_read_rate']
        self.last_battery_t = 0

        # initial status
        self.modem_status = MODEM_ENABLED
        self.battery_level = 0
        self.propagation_time_flag = False
        self.propagation_time = 0

        prefix = self.name + '/'
        # instant message publishers and subscribers
        self.pub_im_in = rospy.Publisher(prefix + TOPIC_IM_IN, AcousticModemPayload, tcp_nodelay=True, queue_size=1)
        self.pub_im_ack = rospy.Publisher(prefix + TOPIC_IM_ACK, AcousticModemAck, tcp_nodelay=True, queue_size=1)
        self.sub_im_out = rospy.Subscriber(prefix + TOPIC_IM_OUT, AcousticModemPayload, self.handle_im_out, tcp_nodelay=True, queue_size=1)

        # synchronous instant message publishers and subscribers
        self.pub_ims_in = rospy.Publisher(prefix + TOPIC_IMS_IN, AcousticModemPayload, tcp_nodelay=True, queue_size=1)
        self.pub_ims_ack = rospy.Publisher(prefix + TOPIC_IMS_ACK, AcousticModemAck, tcp_nodelay=True, queue_size=1)
        self.sub_ims_out = rospy.Subscriber(prefix + TOPIC_IMS_OUT, AcousticModemPayload, self.handle_ims_out, tcp_nodelay=True, queue_size=1)

        # burst message publishers and subscribers
        self.pub_burst_in = rospy.Publisher(prefix + TOPIC_BURST_IN, AcousticModemPayload, tcp_nodelay=True, queue_size=1)
        self.pub_burst_ack = rospy.Publisher(prefix + TOPIC_BURST_ACK, AcousticModemAck, tcp_nodelay=True, queue_size=1)
        self.sub_burst_out = rospy.Subscriber(prefix + TOPIC_BURST_OUT, AcousticModemPayload, self.handle_burst_out, tcp_nodelay=True, queue_size=1)

        # USBL data publishers
        self.pub_usbllong = rospy.Publisher(prefix + TOPIC_USBLLLONG, AcousticModemUSBLLONG, tcp_nodelay=True, queue_size=1)
        self.pub_usblangles = rospy.Publisher(prefix + TOPIC_USBLANGLES, AcousticModemUSBLANGLES, tcp_nodelay=True, queue_size=1)

        # status publisher
        self.pub_status = rospy.Publisher(prefix + TOPIC_STATUS, AcousticModemStatus, tcp_nodelay=True, queue_size=1)

        # services
        self.s_switch = rospy.Service(SRV_SWITCH, BooleanService, self.srv_switch)

        # optional info
        if self.verbose:
            t_pri = rospy.Timer(rospy.Duration(0.5), self.print_info)

    def handle_im_out(self, msg):
        """
        ROS callback
        Send an instant message to the device
        :param msg: ROS message, type: AcousticModemPayload
        """
        if self.modem_status:
            err_code, cmd_sent = self.modem.add_im_to_buffer(msg.address, msg.ack, msg.payload)
            self.eval_error_code(err_code, cmd_sent)
            if err_code in [ed.OK, ed.WARN_ACK_BUSY]:
                self.sent_im_ack_cnt += 1
            self.modem.send_line()
        else:
            rospy.warn("%s: Modem is disabled! Command ignored" % self.name)

    def handle_ims_out(self, msg):
        # Publish to serial port
        # To be done
        pass

    def handle_burst_out(self, msg):
        """
        ROS callbackself.propagation_time
        Send an burst message to the device
        :param msg: ROS message, type: AcousticModemPayload
        """
        if self.modem_status:
            err_code, cmd_sent = self.modem.add_burst_to_buffer(msg.address, msg.payload)
            self.eval_error_code(err_code, cmd_sent)
            # print self.sent_burst_cnt
            if err_code in [ed.OK, ed.WARN_ACK_BUSY]:
                self.sent_burst_cnt += 1
        else:
            rospy.warn("%s: Modem is disabled! Command ignored" % self.name)

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
        msg = AcousticModemStatus()
        msg.header.stamp = rospy.Time.now()
        msg.status = MODEM_STATUS[self.modem_status]
        msg.battery_level = self.battery_level
        # print 'im_ack_cnt', self.im_ack_cnt
        # print 'burst_ack_cnt', self.burst_ack_cnt

        if self.sent_im_ack_cnt > 0 and self.sent_burst_cnt > 0:
            info = {
                'im_ack_success_rate': str(self.im_ack_cnt/self.sent_im_ack_cnt),
                'im_burst_rate': str(self.burst_ack_cnt/self.sent_burst_cnt),
                'total_sent': str(self.sent_burst_cnt + self.sent_im_ack_cnt),
                'total_received': str(self.received_cnt)
            }
            msg.info = [KeyValue(key, value) for key, value in info.items()]

        self.pub_status.publish(msg)

    # Parsing methods
    def parse_acoustic_bm(self, tokens):
        """
        This method parse an incoming burst message and publish its content on a dedicated topic
        :param tokens: list of message fields
        """
        msg = AcousticModemPayload()
        msg.header.stamp = rospy.Time.now()

        msg.address = int(tokens[2])
        msg.bitrate = int(tokens[4])
        msg.rssi = float(tokens[5])
        msg.integrity = float(tokens[6])
        msg.propagation_time = int(tokens[7])
        self.propagation_time = int(tokens[7])
        # print 'Distance', self.propagation_time * 1500 * 10**-6
        msg.relative_velocity = float(tokens[8])

        msg.payload = ','.join(tokens[9:])

        rospy.loginfo('%s: Received burst message from node %s, payload: %s, info: %s' % (self.name, msg.address, repr(msg.payload), repr(msg.info)))
        self.received_cnt += 1
        self.pub_burst_in.publish(msg)

    def parse_acoustic_im(self, tokens):
        """
        This method parses an incoming instant message and publish its content on a dedicated topic
        :param tokens: list of message fields
        """
        msg = AcousticModemPayload()
        msg.header.stamp = rospy.Time.now()

        msg.address = int(tokens[2])
        msg.duration = int(tokens[5])
        msg.rssi = float(tokens[6])
        msg.integrity = float(tokens[7])
        msg.relative_velocity = float(tokens[8])
        msg.payload = ','.join(tokens[9:])

        rospy.loginfo('%s: Received IM from node %s, payload: %s, info: %s' %(self.name, msg.address, repr(msg.payload), repr(msg.info)))
        self.received_cnt += 1
        self.pub_im_in.publish(msg)

    def parse_usbllong(self, tokens):
        """
        method which will generate a ros message from USBL data
        :param tokens: list of message fields
        """
        if len(tokens) != 17:
            rospy.logerr("%s: USBLLONG message fields count doesn't match" % self.name)
            return
        usblmsg = AcousticModemUSBLLONG()
        usblmsg.header.stamp = rospy.Time.now()

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
        self.received_cnt += 1
        self.pub_usbllong.publish(usblmsg)

    def parse_usblangles(self, tokens):
        """
        method which will generate a ros message from USBL data
        :param tokens: list of message fields
        """
        if len(tokens) != 14:
            rospy.logerr("%s: USBLANGLES message fields count doesn't match" % self.name)
            return
        usblmsg = AcousticModemUSBLANGLES()
        usblmsg.header.stamp = rospy.Time.now()

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
        self.received_cnt += 1
        self.pub_usblangles.publish(usblmsg)

    def parse_delivered(self, tokens):
        rospy.loginfo("%s: Message delivered to node %s" %(self.name,tokens[1]))
        # print self.modem.last_ack_type
        if self.modem.last_ack_type == "imack":
            self.propagation_time_flag = True  # to check propagation time later
            self.im_ack_cnt += 1
            msg = AcousticModemAck()
            msg.header.stamp = rospy.Time.now()
            msg.ack = True
            self.pub_im_ack.publish(msg)
        elif self.modem.last_ack_type == "burstack":
            self.burst_ack_cnt += 1
            msg = AcousticModemAck()
            msg.header.stamp = rospy.Time.now()
            msg.ack = True
            self.pub_burst_ack.publish(msg)
        self.modem.last_ack_type = ""

    def parse_failed(self, tokens):
        rospy.logwarn("%s: Message delivering failed to node %s" %(self.name,tokens[1]))
        if self.modem.last_ack_type == "imack":
            msg = AcousticModemAck()
            msg.header.stamp = rospy.Time.now()
            msg.ack = False
            self.pub_im_ack.publish(msg)
        elif self.modem.last_ack_type == "burstack":
            msg = AcousticModemAck()
            msg.header.stamp = rospy.Time.now()
            msg.ack = False
            self.pub_burst_ack.publish(msg)
        self.modem.last_ack_type = ""

    def parse_ok(self,tokens):
        pass

    def eval_error_code(self, code, cmd_sent):
        if code == ed.ERROR_DATA_TOO_LONG:
            rospy.logerr("%s: Data too long! Command ignored!" % self.name)
        elif code == ed.ERROR_MODEM_BUSY:
            rospy.logerr("%s: Device busy! Command ignored!" % self.name)
        elif code == ed.WARN_ACK_BUSY:
            rospy.logwarn("%s: Still waiting for previous ack but starting new one")
        elif code == ed.OK:
            # print self.name, cmd_sent
            rospy.loginfo("%s: Command sent: %s" % (self.name, cmd_sent))
            return OK
        return ERROR

    def eval_voltage_scheduler(self):
        time_since_measurement = rospy.Time.now().to_sec() - self.last_battery_t
        if time_since_measurement > self.battery_read_period:
            self.last_battery_t = rospy.Time.now().to_sec()
            code, cmd = self.modem.read_battery_level()
            self.eval_error_code(code, cmd)
            time.sleep(0.1)
            self.modem.read_device_to_buffer()
            try:
                line = self.modem.read_buffer.pop()
                self.battery_level = float(line)
            except ValueError:
                rospy.logwarn("%s: Battery read failed - value not updated. Command-response mismatch?" % self.name)
                self.parse_line(line)
            except IndexError:
                rospy.logwarn("%s: Propagation time read failed - value not updated. Modem not connected?" % self.name)

    def eval_propagation_time(self):
        if self.propagation_time_flag:
            code, cmd = self.modem.read_propagation_time()
            self.eval_error_code(code, cmd)
            time.sleep(0.1)
            self.modem.read_device_to_buffer()
            try:
                line = self.modem.read_buffer.pop()
                self.propagation_time = float(line)
                print 'Distance', self.propagation_time * 1500 * 10**-6
                self.propagation_time_flag = False
            except ValueError:
                rospy.logwarn("%s: Propagation time read failed - value not updated. Command-response mismatch?" % self.name)
                self.parse_line(line)
            except IndexError:
                rospy.logwarn("%s: Propagation time read failed - value not updated. Modem not connected?" % self.name)

    def eval_modem_status(self):
        time_since_status = rospy.Time.now().to_sec() - self.last_status_t
        if time_since_status > self.status_period:
            self.last_status_t = rospy.Time.now().to_sec()
            self.send_status()

    def parse_line(self, line):
        """
        Function called in the main loop, just read what is coming on the port and parse it using a dictionary
        The frequency is set by the parameter read_rate
        """
        rospy.loginfo('%s: Received line: %s' % (self.name, repr(line)))
        tokens = line.split(',')
        first_token = tokens[0]  # command
        if first_token in self.dict_messages_types.keys():
            self.dict_messages_types[first_token](tokens)
        else:
            rospy.logwarn('%s: Failed to parse line: %s' % (self.name, line))

    def parse_buffer(self):
        self.modem.read_device_to_buffer()
        while len(self.modem.read_buffer) > 0:
            line = self.modem.read_buffer.pop()
            self.parse_line(line)

    def run(self):
        # driver loop
        while not rospy.is_shutdown():
            self.parse_buffer()
            self.eval_voltage_scheduler()
            self.parse_buffer()
            self.eval_propagation_time()
            self.modem.send_line()
            self.eval_modem_status()
            try:
                self.driver_loop.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo('%s shutdown requested ...', self.name)

        self.modem.close()

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

    config = DEFAULT_CONFIG.copy()
    # load global parameters
    param_config = rospy.get_param('~modem_config', {})
    # Update default settings with user specified params
    config.update(param_config)

    # show current settings
    rospy.loginfo('%s modem config: %s', name, config)

    # start vehicle control node
    driver = ModemNode(name, config, verbose=verbose)

    try:
        driver.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s uncaught exception, dying!\n%s', name, tb)


if __name__ == '__main__':
    main()
