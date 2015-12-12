#!/usr/bin/env bash

# usage
print_usage() {
    echo "Usage: $(basename $0) <address> <payload> <rate>"
    echo "Send the <payload> to acoustic modem with address <address>."
    echo ""
    echo "Mandatory arguments:"
    echo "  <address>: unsigned int from range (0, 255)"
    echo "  <payload>: a string not longer 1024 chars"
    echo "  <rate>: in Hz"
    echo ""
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

# vars
ADDRESS=$1
PAYLOAD=$2
RATE=$3

rostopic pub -r ${RATE} /modem/burst/out vehicle_interface/AcousticModemPayload "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
msg_id: 0
address: ${ADDRESS}
ack: false
bitrate: 0
rssi: 0.0
integrity: 0.0
propagation_time: 0
duration: 0
relative_velocity: 0.0
payload: '${PAYLOAD}'
info:
- {key: '', value: ''}"


