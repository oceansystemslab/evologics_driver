#!/usr/bin/env python

# ---------
# Author: Gordon Frost
# Email: gwf2@hw.ac.uk
# Date: 20/05/16
# Notes: - Sender and receiver node reads from the same text file, sending
#		"packets" of text in a TDMA-like schedule
#		- Configuration is mostly done in below Dicts but some params like
#			like start time and transmitter flag can be set from terminal
#			Terminal parameters take precedence
#		- Results are logged in a simple CSV file of format: "time slot num"\t"0 or 1(failed or succeded)
#----------
import roslib; roslib.load_manifest("evologics_driver")
import rospy
import time # time.localtime() = (year, month, day, hour, 
			 #min, sec, weekday, day of the year, daylight saving flag)
import os
from optparse import OptionParser

from vehicle_interface.msg import AcousticModemPayload

TOPIC_IM_IN = '/modem/im/in'
TOPIC_IM_OUT = '/modem/im/out'
TOPIC_BURST_IN = '/modem/burst/in'
TOPIC_BURST_OUT = '/modem/burst/out'

# TODO: Method of setting the start time remotely through an IM?

BURST_MSG = {
	"packet_size": 512, # bytes
	"ID": 0,	# Local modem ID number
	"IDR": 1,	# Remote modem ID number
	"rx_msg_topic": TOPIC_BURST_IN,
	"send_msg_topic": TOPIC_BURST_OUT
	}
INSTANT_MSG = {
	"packet_size": 10, # bytes                                                             
	"ID": 0,	# Local modem ID number
	"IDR": 1,	# Remote modem ID number
	"rx_msg_topic": TOPIC_IM_IN,
	"send_msg_topic": TOPIC_IM_OUT
}

CONFIG = {
	"transmitting":	True,
	"remote_config": False,
	"start_time": 1519,
	"num_slots": 	5, # number of packets to transmit
	"period": 		10, # seconds
	"msg_type": 	BURST_MSG,	# burst or instant
	"filepath": 	"test_file.txt",
	"logging_base_directory": "/tmp/modem_per_tests"
}
verbose = False
parser = OptionParser()
parser.add_option("--transmit", action="store_true", dest="transmitting")
parser.add_option("--receive", action="store_false", dest="transmitting")
parser.add_option("--time", action="store", type=int, dest="start_time")
                  
(options, args) = parser.parse_args()
print(options)

# Update the CONFIG options with the options taken from the command line
# TODO: find more automated way to achieve this
if options.transmitting is not None and options.start_time is not None:
	CONFIG["transmitting"] = options.transmitting
	CONFIG["start_time"] = options.start_time

# File which contains the test data, chunks of which are sent each timeslot 
f = open(CONFIG["filepath"], "r")

def on_rospyShutdown():
	global f
	f.close()

class Transmitter(object):
	def __init__(self):
		self.acoustic_msg_pub = rospy.Publisher(CONFIG["msg_type"]["send_msg_topic"], AcousticModemPayload)

	def send_msg(self, msg_idx, remote_address, ack, packet):
		acoustic_msg = AcousticModemPayload()
		acoustic_msg.msg_id = msg_idx
		acoustic_msg.address = remote_address
		acoustic_msg.ack = ack
		acoustic_msg.payload = packet
		# Send the packet to the Evologics driver which will send to modem
		self.acoustic_msg_pub.publish(acoustic_msg)

class Receiver(object):
	def __init__(self):
		acoustic_msg_sub = rospy.Subscriber(CONFIG["msg_type"]["rx_msg_topic"], AcousticModemPayload, self.rx_acoustic_msgCallback)
		self.last_received_packet = None
	def new_timeslot(self):
		self.last_received_packet = None
	def rx_acoustic_msgCallback(self, msg):
		self.last_received_packet = msg.payload
	def check_timeslot_reception(self, ground_truth_packet):
		if self.last_received_packet == ground_truth_packet:
			return True
		else:
			return False

if __name__=='__main__':
	global f
	rospy.init_node("per_test_node")
	script_start_time = time.localtime()
	if CONFIG["transmitting"]:
		print("TRANSMITTER")
	else:
		print("RECEIVING")
	
	print("Script Started at: {0}:{1}".format(script_start_time[3], script_start_time[4]))
	print("Programmed Start Time: {0}".format(CONFIG["start_time"]))
	
	# Transmitter or Receiver objects set up Pubs and Subs for 
	# comms to Evologics Driver in their __init__
	if CONFIG["transmitting"]:
		modemNode = Transmitter()
	else:
		modemNode = Receiver()
		# Set up logging file
		
		if not os.path.exists(CONFIG["logging_base_directory"]):
			os.makedirs(CONFIG["logging_base_directory"])
		log_filename = "per_test_{0}.log".format(time.time())
		f_results = open("{0}/{1}".format(CONFIG["logging_base_directory"], log_filename), "w", 1)
	
	
	
	# Providing start time is not in past, wait till start time
	while (not rospy.is_shutdown()):
		print("Waiting for start time .... {0}".format(int("{0}{1}".format(time.localtime()[3], time.localtime()[4]))))
		first_time_slot_start_time = time.localtime()
		hour = str(first_time_slot_start_time[3]).zfill(2)
		minute = str(first_time_slot_start_time[4]).zfill(2)
		first_time_slot_start_time = int("{0}{1}".format(hour, minute))
		print("current time: {0}".format(first_time_slot_start_time))
		if first_time_slot_start_time >= CONFIG["start_time"]:
			break
		rospy.sleep(0.1)
		
	# Save the time, unix timestamp, of the first timeslot
	# Future timeslots are timed based on this unix time by adding CONFIG["period"]
	start_time_unix = time.time()
	
	while not rospy.is_shutdown():		
		# Step through the defined number of TDMA time slots
		for tx_idx in range(CONFIG["num_slots"]):
			print("----------")
			print(time.localtime())
			
			# Data and Transmission/Reception stuff now
			# Read the number of bytes configured to be sent each timeslot
			packet = f.read(CONFIG["msg_type"]["packet_size"])
			if CONFIG["transmitting"]:
				modemNode.send_msg(tx_idx, CONFIG["msg_type"]["IDR"],
									False, packet)
				print("SENDING Packet {0}: {1}".format(tx_idx, packet[0:20]))
			else:
				# reset the last received msg to be None
				# Check is performed at the end of a timeslot due to 
				# latencies in acoustic transmission
				modemNode.new_timeslot()
			
			
			
			# Timing Stuff Now
			# Wait for "period" amount of time between transmissions
			while not rospy.is_shutdown() and time.time() < start_time_unix + CONFIG["period"]:
				rospy.sleep(0.1)
			# Make sure to kill the whole process at this point if rospy
			# is stopped
			if rospy.is_shutdown():
				exit(0)
			start_time_unix = time.time()
			
			# At the end of the timeslot, check if we received the packet correctly
			if not CONFIG["transmitting"]:
				received_ok = modemNode.check_timeslot_reception(packet)
				print("RECEIVING Packet {0}, RX'd OK: {1}".format(tx_idx, received_ok))
				f_results.write("{0}\t{1}\n".format(tx_idx, int(received_ok)))
		rospy.signal_shutdown("Test Completed")
	rospy.spin()
