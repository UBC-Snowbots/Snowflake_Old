import pytest

from ctypes import sizeof
from jaus.general_transport_header import (
	GeneralTransportHeaderStructure,
	parse_sequence_number,
	parse
)

def test_header_from_bytes_with_striped_bytes():
	data = '\xF0'*sizeof(GeneralTransportHeaderStructure)
	
	hdr = GeneralTransportHeaderStructure()
	hdr.from_bytes(data)
	
	assert hdr.message_type == 0b111100
	assert hdr.HC_flags == 0b00
	assert hdr.data_size == 0b1111000011110000
	assert hdr.HC_number == 0b11110000
	assert hdr.HC_length == 0b11110000
	assert hdr.priority == 0b11
	assert hdr.broadcast == 0b11
	assert hdr.ack_nack == 0b00
	assert hdr.data_flags == 0b00
	assert hdr.destination_id == 0xF0F0F0F0
	assert hdr.source_id == 0xF0F0F0F0

def test_header_from_bytes_with_striped_half_bytes():
	data = '\x66'*sizeof(GeneralTransportHeaderStructure)
	
	hdr = GeneralTransportHeaderStructure()
	hdr.from_bytes(data)
	
	assert hdr.message_type == 0b011001
	assert hdr.HC_flags == 0b10
	assert hdr.data_size == 0x6666
	assert hdr.HC_number == 0x66
	assert hdr.HC_length == 0x66
	assert hdr.priority == 0b01
	assert hdr.broadcast == 0b10
	assert hdr.ack_nack == 0b01
	assert hdr.data_flags == 0b10
	assert hdr.destination_id == 0x66666666
	assert hdr.source_id == 0x66666666

def test_header_from_bytes_incorrect_data_size():
	hdr = GeneralTransportHeaderStructure()
	# data too short
	with pytest.raises(ValueError):
		hdr.from_bytes('')
	# data too long
	with pytest.raises(ValueError):
		hdr.from_bytes(
			'a'*(
				sizeof(GeneralTransportHeaderStructure) + 3))

def test_parse_sequence_number():
	"""
		Assert correct parsing of 2 byte integers, reject malformed
		input.
	"""
	assert parse_sequence_number('\x12\x34') == 0x1234
	with pytest.raises(ValueError):
		parse_sequence_number('a')
	with pytest.raises(ValueError):
		parse_sequence_number('aaa')
