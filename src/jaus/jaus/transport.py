from .general_transport_header import parse as parse_transport_header

class Packet(object):
	# Sequencing constants
	ONLY_PART = 0b00
	FIRST_PART = 0b01
	MIDDLE_PART = 0b10
	LAST_PART = 0b11
	
	def __init__(self, header, contents, sequence_number):
		self.header = header
		self.contents = contents
		self.sequence_number = sequence_number

def parse_packets(data):
	result = []
	while data:
		(
			header,
			contents,
			sequence_number,
			data) = parse_transport_header(data)
		result.append(Packet(header, contents, sequence_number))
	return result
