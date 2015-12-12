__author__ = 'niccolo'
import struct


class Container(object):
    def __init__(self, data_list):
        if len(data_list) != 7:
            print "input error"
            raise NameError('Input Error: list length must be 7, given ' + str(len(data_list)))
        self.measurement_time = data_list[0]
        self.sender = data_list[1]
        self.receiver = data_list[2]
        self.latitude = data_list[3]
        self.longitude = data_list[4]
        self.depth = data_list[5]
        self.accuracy = data_list[6]

    def get_list(self):
        return [self.time, self.sender, self.receiver, self.latitude, self.longitude, self.depth, self.accuracy]


class Packer(object):
    def pack(self, input):
        if not isinstance(input, Container) :
            print "input error"
            raise NameError('Input Error: Container required')
        return struct.pack("!fBBddff", input.measurement_time,
                                       input.sender,
                                       input.receiver,
                                       input.latitude,
                                       input.longitude,
                                       input.depth,
                                       input.accuracy)

    def unpack(self, input):
        if type(input) != str:
            print "input error"
            return False
        return Container(struct.unpack("!fBBddff", input))
    
    def length(self):
        return struct.calcsize("!fBBddff")
