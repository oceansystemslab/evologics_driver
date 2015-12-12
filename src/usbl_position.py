#!/usr/bin/env python
# license removed for brevity

import rospy
import roslib
roslib.load_manifest("evologics_driver")
from std_msgs.msg import String
import usbl_packer as up
from vehicle_interface.msg import AcousticModemUSBLLONG, AcousticModemPayload, AcousticLocalization
from auv_msgs.msg import NavSts, DecimalLatLon
import numpy as np
import math

# USBL data
TOPIC_USBLLLONG = "modem/usbllong"
TOPIC_LOCALIZATION = "modem/localization"

# Modem msg
TOPIC_IM_OUT = 'modem/im/out'
TOPIC_IM_IN = 'modem/im/in'

# NavStatus
TOPIC_NAV = "nav/nav_sts"

# Params
PARAM_NODES_LIST = "modem/nodes_list"
PARAM_NODE = "modem/node"
PARAM_SLOT_DURATION = "modem/slot_duration"
PARAM_BIM_TIME = "modem/maximum_bim_time"
PARAM_IM_TIME = "modem/maximum_im_time"
PARAM_IS_THIS_USBL = "modem/is_this_usbl"
PARAM_START_TIME = "tdma/start_time"
# Header for the payload
MSG_HEADER = "USBL:"



def deg_to_rad(deg):
    return deg * np.pi / 180.0


def rad_to_deg(rad):
    return rad / np.pi * 180.0


def ned2lla(ned,  lla0):
    # Coordinate conversion from North East Down (m) to Latitude Longitude Altitude (Degree)
    lla = np.array([0.0, 0.0, 0.0])
    lla0_rad = deg_to_rad(lla0)
    a = 6378137.0
    f = 1.0 / 298.257223563
    rn = a / np.sqrt(1 - (2 * f - f * f) * np.sin(lla0_rad[0]) * np.sin(lla0_rad[0]))
    rm = rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * np.sin(lla0_rad[0]) * np.sin(lla0_rad[0]))
    lla[0] = rad_to_deg((lla0_rad[0] + np.arctan2(1, rm) * ned[0]))
    lla[1] = rad_to_deg((lla0_rad[1] + np.arctan2(1, rn * np.cos(lla0_rad[0])) * ned[1]))
    lla[2] = -ned[2] + lla0[2]
    return lla


class PositionExchanger():
    def __init__(self):

        # Read parameters from the server
        self.nodes_list = self.get_param(PARAM_NODES_LIST,[9, 5])
        self.node = self.get_param(PARAM_NODE, 10)
        self.slot_duration = self.get_param(PARAM_SLOT_DURATION,20)
        self.time_for_bim = self.get_param(PARAM_BIM_TIME,1)
        self.time_for_im = self.get_param(PARAM_IM_TIME,3)
        self.is_this_usbl = self.get_param(PARAM_IS_THIS_USBL, True)
        self.start_time = self.get_param(PARAM_START_TIME, rospy.get_time())

        self.bim_queue = list()
        self.flag_busy = False
        self.data_packer = up.Packer()
        self.flag_initialized = False

        # Remove our node from the list
        if self.node in self.nodes_list:
            self.nodes_list.remove(self.node)

        self.actual_pos = np.array([0.0, 0.0, 0.0])

        # ROS Subscribers and Publishers
        self.sub_usbl_fix = rospy.Subscriber(TOPIC_USBLLLONG, AcousticModemUSBLLONG, self.handle_usbl_fix, tcp_nodelay=True, queue_size=10)
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, tcp_nodelay=True, queue_size=1)
        self.sub_msg = rospy.Subscriber(TOPIC_IM_IN, AcousticModemPayload, self.handle_im, tcp_nodelay=True, queue_size=1)

        self.pub_msg = rospy.Publisher(TOPIC_IM_OUT, AcousticModemPayload, tcp_nodelay=True, queue_size=1)
        self.pub_loc = rospy.Publisher(TOPIC_LOCALIZATION, AcousticLocalization, tcp_nodelay=True, queue_size=1)

    def get_param(self, name, default):
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            return default

    def loop(self):
        if not self.is_this_usbl:
            return

        if self.flag_busy:
            return

        # # check if the time is time for sending positional info to other vehicle
        # if self.check_time("SEND_INFO") and len(self.bim_queue) != 0:
        #     rospy.loginfo("Current action: SEND_INFO")
        #     self.send_info(self.bim_queue.pop(0))
        # # or if the time is for getting USBL localisation data of the other vehicles
        # elif self.check_time("GET_LOCALIZATION") and len(self.nodes_list) != 0:
        #     rospy.loginfo("Current action: GET_LOCALIZATION of node: " + str(self.nodes_list[0]))
        #     self.send_request()
        # # or if the time is for the world model/other TDMA-based platforms comms so USBL tracker shouldn't speak
        # else:
        #     return

        # ----------
        # Wait until next time slot
        while not self.check_timeV2() and not rospy.is_shutdown():
            print("Waiting till next timeslot")
            rospy.sleep(0.5)
        self.bim_queue = []
        # Time to wait at beginning of tracker transmission timeslot
        rospy.sleep(1)

        # get positions
        for node in self.nodes_list:
            rospy.loginfo("Current action: GET_LOCALIZATION of node: " + str(node))
            self.send_requestV2(node)
            rospy.sleep(2)

        print("Burst Queue:")
        print(self.bim_queue)

        # Send the broadcast msgs containing the vehicles positions to the fleet
        for broadcast_data in self.bim_queue:
            self.send_info(broadcast_data)
            rospy.sleep(2)


        # Wait for remainder of timeslot
        while self.check_timeV2() and not rospy.is_shutdown():
            rospy.sleep(0.5)

        # now send broadcast msg with the positions

        # if len(self.bim_queue) != 0:
        #     if not self.check_time("SEND_INFO"):
        #         return
        #     rospy.loginfo("Current action: SEND_INFO")
        #     self.send_info(self.bim_queue.pop(0))

        # elif len(self.nodes_list) != 0:
        #     if not self.check_time("GET_LOCALIZATION"):
        #         return
        #     rospy.loginfo("Current action: GET_LOCALIZATION of node: " + str(self.nodes_list[0]))
        #     self.send_request()
    def send_requestV2(self, node):
        # send an im to a device to get his position
        # self.flag_busy = True
        self.timer = rospy.Timer(rospy.Duration(0, int(self.time_for_im * 1e9)), self.timer_callback, oneshot= True)
        # self.nodes_list.append(node)
        # create and send the message
        msg = AcousticModemPayload()
        msg.ack = True
        msg.address = node
        msg.payload = 'A'
        self.pub_msg.publish(msg)


    def send_request(self):
        # send an im to a device to get his position
        self.flag_busy = True
        self.timer = rospy.Timer(rospy.Duration(0, int(self.time_for_im * 1e9)), self.timer_callback, oneshot= True)
        current_node = self.nodes_list.pop(0)
        self.nodes_list.append(current_node)
        # create and send the message
        msg = AcousticModemPayload()
        msg.ack = True
        msg.address = current_node
        msg.payload = 'A'
        self.pub_msg.publish(msg)

    def send_info(self, data):
        # send a broadcast im to exchange the information stored
        print("Sending Broadcast IM with nav data...")
        self.flag_busy = True
        self.timer = rospy.Timer(rospy.Duration(0, int(self.time_for_bim * 1e9)), self.timer_callback, oneshot=True)
        # create and send the message
        msg = AcousticModemPayload()
        msg.ack = False
        msg.address = 255
        msg.payload = MSG_HEADER + self.data_packer.pack(data)
        self.pub_msg.publish(msg)

    def check_timeV2(self):
        # check if we are in the right part of the time-slot
        safe_time = 2.0
        current_time = rospy.get_time()
        time_from_start = (current_time-self.start_time)
        print("Time from start: {0}".format(time_from_start))
        print("Remainder: {0}".format(math.floor(time_from_start) % self.slot_duration))
        half_timeslot_condition = math.floor(time_from_start) % self.slot_duration > (self.slot_duration / 2) - 1
        safe_transmission_time_condition = math.floor(time_from_start) % (self.slot_duration) < (self.slot_duration) - safe_time
        print([half_timeslot_condition, safe_transmission_time_condition])
        print("Time Check at: {0}; {1}".format(time_from_start, half_timeslot_condition and safe_transmission_time_condition))

        return (half_timeslot_condition and safe_transmission_time_condition)

    def check_time(self, action):
        # check if we are in the right part of the time-slot
        if action == "GET_LOCALIZATION":
            safe_time = self.time_for_im
        elif action == "SEND_INFO":
            safe_time = self.time_for_bim
        else:
            rospy.logerr("Unknown action")
            return False
        current_time = rospy.get_time()
        time_from_start = (current_time-self.start_time)
        print("Time from start: {0}".format(time_from_start))
        print("Remainder: {0}".format(math.floor(time_from_start) % self.slot_duration))
        half_timeslot_condition = math.floor(time_from_start) % self.slot_duration > (self.slot_duration / 2) - 1
        safe_transmission_time_condition = math.floor(time_from_start) % (self.slot_duration) < (self.slot_duration) - safe_time
        print([half_timeslot_condition, safe_transmission_time_condition])
        print("Time Check at: {0}; {1}".format(time_from_start, half_timeslot_condition and safe_transmission_time_condition))
        # return (current_time - math.floor( current_time / self.slot_duration ) * self.slot_duration) > (self.slot_duration / 2.0) - safe_time
        # return

        return (half_timeslot_condition and safe_transmission_time_condition)

    def timer_callback(self, event):
        self.flag_busy = False

    def handle_im(self, msg):
        # parse the im message
        if not (msg.payload.find(MSG_HEADER) == 0):
            return
        payload = msg.payload.replace(MSG_HEADER, "")
#        if not (len(payload) == self.data_packer.length()):
#            rospy.logerr("USBL payload error: size mismatch")
#            return
        packet = self.data_packer.unpack(payload)
        print [packet.measurement_time,
                   packet.receiver,
                   packet.sender,
                   packet.latitude,
                   packet.longitude,
                   packet.depth,
                   packet.accuracy]
        if packet.receiver == self.node:
            # Send a message with localization info
            rospy.loginfo("Reiceived localization message")
            out_msg = AcousticLocalization()
            out_msg.measurement_time = packet.measurement_time
            out_msg.sender_address = packet.sender
            out_msg.latitude = packet.latitude
            out_msg.longitude = packet.longitude
            out_msg.depth = packet.depth
            out_msg.accuracy = packet.accuracy
            self.pub_loc.publish(out_msg)
        else:
            rospy.loginfo("Reiceived localization message of another node")

    def handle_nav(self, msg):
        self.actual_pos = np.array([msg.global_position.latitude, msg.global_position.longitude, 0])
        self.flag_initialized = True

    def handle_usbl_fix(self, msg):
        """
        :param msg: receives USBLANGLE msgs through the ROS evologics driver topic
        :return: appends the msgs data to a queue which is sent every slot period, T, seconds
        """
        if not self.flag_initialized:
            rospy.logerr("Node not yet initialized, localization dropped")
            return
        ned = np.array([ msg.N, msg.E, -msg.U])
        tgt_position = ned2lla(ned, self.actual_pos)
        self.bim_queue.append(up.Container([msg.measurement_time,
                                           self.node,
                                           msg.remote_address,
                                           tgt_position[0],
                                           tgt_position[1],
                                           tgt_position[2],
                                           msg.accuracy]))
        self.flag_busy = False
        if type(self.timer) is rospy.Timer:
            if self.timer.is_alive:
                self.timer.shutdown()
                print [msg.measurement_time,
                        self.node,
                        msg.remote_address,
                        tgt_position[0],
                        tgt_position[1],
                        tgt_position[2],
                        msg.accuracy]


def main():
    rospy.init_node('usbl_position', anonymous=True)
    exchanger = PositionExchanger()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        exchanger.loop()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
