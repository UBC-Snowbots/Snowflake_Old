import pytest

from jaus.transport import Packet
from jaus.combinator import (
	Assembler,
	PacketAlreadyAddedError,
	split_into_runs,
	extract_packet_sequences,
	merge_packet_sequence,
	group_packets_by_sequence
)

from mock import Mock, MagicMock, sentinel, patch, call

@pytest.mark.parametrize('input,output', [
	([1,2,3,4,5], [[1,2,3,4,5]]),
	([], []),
	([1,2,4,5], [[1,2],[4,5]]),
	([1,3], [[1],[3]])
])
def test_split_into_runs(input, output):
	assert list(split_into_runs(input)) == output

ONLY_PART = Mock(name='ONLY_PART')
ONLY_PART.header.data_flags = Packet.ONLY_PART
FIRST_PART = Mock(name='FIRST_PART')
FIRST_PART.header.data_flags = Packet.FIRST_PART
MIDDLE_PART = Mock(name='MIDDLE_PART')
MIDDLE_PART.header.data_flags = Packet.MIDDLE_PART
LAST_PART = Mock(name='LAST_PART')
LAST_PART.header.data_flags = Packet.LAST_PART

@pytest.mark.parametrize('input, output', [
	([MIDDLE_PART], []),
	([LAST_PART], []),
	([FIRST_PART], []),
	([LAST_PART, MIDDLE_PART], []),
	([MIDDLE_PART, LAST_PART], []),
	([FIRST_PART, MIDDLE_PART], []),
	([FIRST_PART, FIRST_PART], []),
	([ONLY_PART, ONLY_PART], [[ONLY_PART], [ONLY_PART]]),
	([FIRST_PART, LAST_PART], [[FIRST_PART, LAST_PART]]),
	([MIDDLE_PART, FIRST_PART, LAST_PART], [[FIRST_PART, LAST_PART]]),
	([FIRST_PART, MIDDLE_PART, LAST_PART], [[FIRST_PART, MIDDLE_PART, LAST_PART]]),
	([FIRST_PART, LAST_PART, FIRST_PART, LAST_PART], [[FIRST_PART, LAST_PART], [FIRST_PART, LAST_PART]]),
	([FIRST_PART, ONLY_PART, LAST_PART], [[ONLY_PART]])
])
def test_extract_packet_sequences_noops(input, output):
	assert list(extract_packet_sequences(input)) == output

def test_merge_packet_sequence_error_on_empty():
	with pytest.raises(ValueError):
		merge_packet_sequence([])

def test_merge_packet_sequence():
	a = Mock(name='a')
	a.header.source_id = sentinel.source_id
	a.header.priority = sentinel.priority
	a.contents = b'abc'
	b = Mock(name='b')
	b.header.source_id = sentinel.source_id
	b.header.priority = sentinel.priority
	b.contents = b'def'
	
	message = merge_packet_sequence([a,b])
	assert message.contents == b'abcdef'
	assert message.priority == sentinel.priority
	assert message.source_id == sentinel.source_id

def test_assembler_add_packet():
	a = Assembler()
	packet = Mock(name='packet')
	packet.sequence_number = 1
	a.add_packet(packet)
	
	assert a._unclaimed.items() == [(1, packet)]
	
	with pytest.raises(PacketAlreadyAddedError):
		a.add_packet(packet)

@patch('jaus.combinator.extract_packet_sequences')
@patch('jaus.combinator.split_into_runs')
def test_group_packets_by_sequence(split_into_runs, extract_packet_sequences):
	packets_by_seq_num = {
		1: sentinel.a,
		2: sentinel.b,
		4: sentinel.c
	}
	split_into_runs.return_value = [[1, 2], [4]]
	extract_packet_sequences.side_effect = [
		[[sentinel.a], [sentinel.b]],
		[[sentinel.c]]]
	
	assert group_packets_by_sequence(packets_by_seq_num) == [
		[sentinel.a],
		[sentinel.b],
		[sentinel.c]]
	
	split_into_runs.assert_called_once_with([1,2,4])
	assert extract_packet_sequences.call_args_list == [
		call([sentinel.a, sentinel.b]),
		call([sentinel.c])
	]

@patch('jaus.combinator.merge_packet_sequence')
@patch('jaus.combinator.group_packets_by_sequence')
def test_assembler_pop_complete_messages(group_packets_by_sequence, merge_packet_sequence):
	assembler = Assembler()
	unclaimed = MagicMock(name='unclaimed')
	assembler._unclaimed = unclaimed
	
	a = Mock(name='a')
	b = Mock(name='b')
	
	group_packets_by_sequence.return_value = [
		[a, b]
	]
	merge_packet_sequence.side_effect = [
		sentinel.message
	]
	
	assert assembler.pop_complete_messages()
	assert group_packets_by_sequence.call_args_list == [
		call(unclaimed)
	]
	assert unclaimed.__delitem__.call_args_list == [
		call(a.sequence_number),
		call(b.sequence_number)
	]
	assert merge_packet_sequence.call_args_list == [
		call([a,b])
	]
