#! /usr/bin/python

"""
ROS node which converts joel world model, osl_world_model, AcousticModemData
msgs into the ones used for the Evologics modem driver
Author: Gordon Frost; email: gwf2@hw.ac.uk
date: 05/02/2015
"""
# TODO: Add Radio Modem switch in here when vehicles known to be on surface

import roslib;roslib.load_manifest("evologics_driver")
import rospy

from vehicle_interface.msg import AcousticModemData
from vehicle_interface.msg import AcousticModemPayload

global wm_incoming_pub, evologics_pub

def wmOutgoingMsg(msg):
    """ method must convert from AcousticModemData uint8[]]
    to AcousticModemPayload string 
    """
    global evologics_pub
    rospy.loginfo("WM outgoing msg being converted and sent to Evologics driver")
    
    uint8_payload = msg.payload
    converted_payload = str(uint8_payload)
    
    # populate new msg and send to evologics modem driver
    converted_msg = AcousticModemPayload()
    converted_msg.address = 5
    converted_msg.payload = converted_payload
    evologics_pub.publish(converted_msg)
    
def wmIncomingMsg(msg):
    """ method must convert from AcousticModemPayload string to
    AcousticModemData uint8 
    """
    global wm_incoming_pub
    rospy.loginfo("Msg received from Evologics driver, sending to WM")
    string_payload = msg.payload
    converted_payload = list(bytearray(string_payload))
    
    # populate new msg and send to world model
    converted_msg = AcousticModemData()
    converted_msg.payload = converted_payload
    wm_incoming_pub.publish(converted_msg)

if __name__ == '__main__':
    rospy.init_node("world_model_msg_translator")
    global wm_incoming_pub, evologics_pub
    # Topics for World model sending message to Evologics modem for transmission
    wm_outgoing_sub = rospy.Subscriber("/modem/outgoing", AcousticModemData, wmOutgoingMsg)
    evologics_pub    = rospy.Publisher("/modem/burst/out", AcousticModemPayload)
    
    # Topics for incoming acoustic msgs being sent on to the World Model
    wm_incoming_pub    = rospy.Publisher("/modem/incoming", AcousticModemData)
    evologics_sub    = rospy.Subscriber("/modem/burst/in", AcousticModemPayload, wmIncomingMsg)
    
    rospy.spin()
    
