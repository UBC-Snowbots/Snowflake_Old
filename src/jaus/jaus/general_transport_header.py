import struct

from ctypes import (
	c_uint, 
	BigEndianStructure,
	memmove,
	sizeof,
	addressof
)

class GeneralTransportHeaderStructure(BigEndianStructure):
	_pack_ = 1
	_fields_ = [
		("message_type", c_uint, 6),
		("HC_flags", c_uint, 2),
		("data_size", c_uint, 16),
		("HC_number", c_uint, 8),
		("HC_length", c_uint, 8),
		("priority", c_uint, 2),
		("broadcast", c_uint, 2),
		("ack_nack", c_uint, 2),
		("data_flags", c_uint, 2),
		("destination_id", c_uint, 32),
		("source_id", c_uint, 32)
	]
	def from_bytes(self, data):
		if len(data) != sizeof(self):
			raise ValueError(
				"Data of the wrong size passed to "
				"GeneralTransportHeaderStructure: data:{} != input:{}"
				.format(
					len(data),
					sizeof(self)))
		memmove(addressof(self), data, sizeof(self))

SEQUENCE_NUMBER_SIZE = 2
def parse_sequence_number(data):
	if len(data) != SEQUENCE_NUMBER_SIZE:
		raise ValueError(
			"Sequence number record is the wrong size. "
			"input: {} != expected: {}"
			.format(
				len(data),
				SEQUENCE_NUMBER_SIZE))
	return struct.unpack('!H', data)[0]

def parse(data):
	header_size = sizeof(GeneralTransportHeaderStructure)
	
	if len(data) < header_size:
		raise ValueError(
			"Message too small to contain general transport header "
			"structure, size: {} < {}"
			.format(
				len(data),
				header_size))
	
	header = GeneralTransportHeaderStructure()
	header.from_bytes(
		data[0:header_size])
	
	sequence_number_start = header_size + header.data_size
	packet_size = sequence_number_start + SEQUENCE_NUMBER_SIZE
	if packet_size != len(data):
		raise ValueError(
			"Message wrong size to contain advertised data, "
			"message: {}, expected: {}"
			.format(
				len(data),
				packet_size))
	
	contents = data[header_size:sequence_number_start]
	sequence_number = parse_sequence_number(
		data[sequence_number_start:SEQUENCE_NUMBER_SIZE])
	return header, contents, sequence_number
