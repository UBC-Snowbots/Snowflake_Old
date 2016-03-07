import pytest

from jaus.general_transport_header import (
	GENERAL_TRANSPORT_HEADER_SIZE,
	GeneralTransportHeaderStructure,
	swap_short_endianness,
	parse_sequence_number,
	parse_contents,
	parse_header,
	parse
)
from mock import patch, sentinel, Mock

@pytest.mark.parametrize('input,output', [
	(0xFF00, 0x00FF),
	(0x00FF, 0xFF00),
	(0xABCD, 0xCDAB),
	(0xABCDE, 0xDEBC) # truncate oversize values
])
def test_swap_short_endianness(input, output):
	expected = '{:02X}'.format(output)
	actual = '{:02X}'.format(
		swap_short_endianness(input))
	assert actual == expected

def test_header_from_bytes_with_striped_bytes():
	data = '\xF0'*GENERAL_TRANSPORT_HEADER_SIZE
	
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

def test_header_little_endian_packet_length():
	data = '\xAB\xCD'*(GENERAL_TRANSPORT_HEADER_SIZE/2)
	
	hdr = GeneralTransportHeaderStructure()
	hdr.from_bytes(data)
	
	assert hdr.message_type == 0b101010
	assert hdr.HC_flags == 0b11
	assert hdr.data_size == 0xABCD
	assert hdr.HC_number == 0xCD
	assert hdr.HC_length == 0xAB
	assert hdr.priority == 0b11
	assert hdr.broadcast == 0b00
	assert hdr.ack_nack == 0b11
	assert hdr.data_flags == 0b01
	assert hdr.destination_id == 0xABCDABCD
	assert hdr.source_id == 0xABCDABCD

def test_header_from_bytes_with_striped_half_bytes():
	data = '\x66'*GENERAL_TRANSPORT_HEADER_SIZE
	
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
				GENERAL_TRANSPORT_HEADER_SIZE + 3))

def test_parse_header_too_short():
	# too short
	with pytest.raises(ValueError):
		parse_header('abc')

@patch('jaus.general_transport_header.GeneralTransportHeaderStructure')
def test_parse_header(general_transport_header_structure):
	data = 'a'*(GENERAL_TRANSPORT_HEADER_SIZE + 2)
	header = general_transport_header_structure()
	assert parse_header(data) == (header, 'aa')
	header.from_bytes.assert_called_once_with(
		data[:GENERAL_TRANSPORT_HEADER_SIZE])

@patch('jaus.general_transport_header.parse_sequence_number')
@patch('jaus.general_transport_header.parse_contents')
@patch('jaus.general_transport_header.parse_header')
def test_parse(parse_header, parse_contents, parse_sequence_number):
	header = Mock(name='header')
	parse_header.return_value = (
		header,
		sentinel.header_tail)
	parse_contents.return_value = (
		sentinel.contents,
		sentinel.contents_tail)
	parse_sequence_number.return_value = (
		sentinel.sequence_number,
		sentinel.sequence_number_tail)
	
	assert parse(sentinel.data) == (
		header,
		sentinel.contents,
		sentinel.sequence_number,
		sentinel.sequence_number_tail)
	
	parse_header.assert_called_once_with(sentinel.data)
	parse_contents.assert_called_once_with(
		sentinel.header_tail,
		header.data_size)
	parse_sequence_number.assert_called_once_with(
		sentinel.contents_tail)

def test_parse_contents():
	assert parse_contents('abc', 0) == ('', 'abc')
	assert parse_contents('abc', 2) == ('ab', 'c')
	assert parse_contents('abc', 3) == ('abc', '')
	with pytest.raises(ValueError):
		parse_contents('abc', 4)

def test_parse_sequence_number():
	"""
		Assert correct parsing of 2 byte integers, reject malformed
		input.
	"""
	assert parse_sequence_number('\x12\x34') == (0x1234, '')
	assert parse_sequence_number('\x12\x34a') == (0x1234, 'a')
	with pytest.raises(ValueError):
		parse_sequence_number('a')
