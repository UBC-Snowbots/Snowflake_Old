import struct

from ctypes import (
	c_uint, 
	BigEndianStructure,
	memmove,
	sizeof,
	addressof
)

def swap_short_endianness(val):
	"""Swap the endianness of a 16-bit integer."""
	return (
		((val << 8) & 0xFF00) # LSB -> MSB
		| ((val >> 8) & 0xFF)) # MSB -> LSB

class GeneralTransportHeaderStructure(BigEndianStructure):
	_pack_ = 1
	_fields_ = [
		("message_type", c_uint, 6),
		("HC_flags", c_uint, 2),
		# data_size for some reason has to be in opposite endianness
		# to everything else, so the big-endian extraction here is
		# wrong
		("data_size_swapped", c_uint, 16),
		("HC_number", c_uint, 8),
		("HC_length", c_uint, 8),
		("priority", c_uint, 2),
		("broadcast", c_uint, 2),
		("ack_nack", c_uint, 2),
		("data_flags", c_uint, 2),
		("destination_id", c_uint, 32),
		("source_id", c_uint, 32)
	]
	@property
	def data_size(self):
		"""Correct the endianness of data_size on read."""
		return swap_short_endianness(self.data_size_swapped)
	@data_size.setter
	def data_size(self, value):
		self.data_size_swapped = swap_short_endianness(value)

	def from_bytes(self, data):
		"""Read all the fields from the bytes provided"""
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
	sequence_number_data = data[:SEQUENCE_NUMBER_SIZE]
	if len(sequence_number_data) < SEQUENCE_NUMBER_SIZE:
		raise ValueError(
			"Message too small to contain sequence number."
			" {} < {}"
			.format(
				len(sequence_number_data),
				SEQUENCE_NUMBER_SIZE))
	return (
		struct.unpack('!H', data[:SEQUENCE_NUMBER_SIZE])[0],
		data[SEQUENCE_NUMBER_SIZE:]
	)

GENERAL_TRANSPORT_HEADER_SIZE = sizeof(GeneralTransportHeaderStructure)
def parse_header(data):
	if len(data) < GENERAL_TRANSPORT_HEADER_SIZE:
		raise ValueError(
			"Message too small to contain general transport header "
			"structure, size: {} < {}"
			.format(
				len(data),
				GENERAL_TRANSPORT_HEADER_SIZE))
	header = GeneralTransportHeaderStructure()
	header.from_bytes(
		data[:GENERAL_TRANSPORT_HEADER_SIZE])
	return header, data[GENERAL_TRANSPORT_HEADER_SIZE:]

def parse_contents(data, size):
	if size > len(data):
		raise ValueError(
			"Message too small to contain contents."
			" {} < {}"
			.format(
				len(data),
				size))
	return data[:size], data[size:]

def parse(data):
	header, tail = parse_header(data)
	
	contents, tail = parse_contents(tail, header.data_size)
	
	sequence_number, tail = parse_sequence_number(tail)
	
	return header, contents, sequence_number, tail
