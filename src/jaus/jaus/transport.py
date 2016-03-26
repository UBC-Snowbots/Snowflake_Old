import collections as _collections
import enum as _enum

from . import messages as _messages

from .format import Specification, Int, Bytes, Optional, Enum, Consume


class JUDPPacketDataFlags(_enum.Enum):
	SINGLE_PACKET = 0b00
	FIRST_PACKET = 0b01
	NORMAL_PACKET = 0b10
	LAST_PACKET = 0b11

class JUDPPacketHCFlags(_enum.Enum):
	NONE = 0
	REQUESTED = 1
	HC_LENGTH = 2
	COMPRESSED = 3

class JUDPPacketPriority(_enum.Enum):
	LOW = 0
	STANDARD = 1
	HIGH = 2
	SAFETY = 3

class JUDPPacketBroadcastFlags(_enum.Enum):
	# single destination
	NONE = 0
	# local destinations only
	LOCAL = 1
	# all destinations
	GLOBAL = 2

class JUDPPacketACKNACKFlags(_enum.Enum):
	NO_RESPONSE_REQUIRED = 0
	RESPONSE_REQUIRED = 1
	# Responses
	NAK = 2
	ACK = 3

JUDPPacketSpecification = Specification('JUDPPacket', specs=[
	Int('message_type', bits=6),
	# if message type is not 0, bail now
	# as this is not a protocol we know
	Optional(lambda attrs: attrs['message_type']==0, [
		Enum('HC_flags', enum=JUDPPacketHCFlags, bits=2),
		Int('data_size', bytes=2, endianness='le'),
		# these fields only exist if header compression is used
		Optional(lambda attrs: attrs['HC_flags'] != JUDPPacketHCFlags.NONE, [
			Int('HC_number', bytes=1),
			Int('HC_length', bytes=1),
		]),
		Enum('priority', enum=JUDPPacketPriority, bits=2),
		Enum('broadcast', enum=JUDPPacketBroadcastFlags, bits=2),
		Enum('ack_nack', enum=JUDPPacketACKNACKFlags, bits=2),
		Enum('data_flags', enum=JUDPPacketDataFlags, bits=2),
		Int('destination_id', bytes=4, endianness='le'),
		Int('source_id', bytes=4, endianness='le'),
		Bytes('contents', length=lambda attrs: attrs['data_size']),
		Int('sequence_number', bytes=2, endianness='le'),
	]),
])
JUDPPacket = JUDPPacketSpecification.type

JUDPPayloadSpecification = Specification('JUDPPayload', specs=[
	Int('transport_version', bytes=1),
	# We only support transport version 2 right now...
	Optional(lambda attrs: attrs['transport_version'] == 2, [
		Consume('packets', JUDPPacketSpecification),
	]),
])
JUDPPayload = JUDPPayloadSpecification.type

class DuplicatePacket(Exception):
	"""A second packet with the same sequence number was added."""

class MessageReassembler(object):
	def __init__(self):
		self.packets_received = {}
		self.complete_packets = set()
		self.start_packets = set()
	
	def add_packet(self, packet):
		if packet.sequence_number in self.packets_received:
			raise DuplicatePacket(
				"Duplicate packet {}.".format(packet.sequence_number))
		
		self.packets_received[packet.sequence_number] = packet
		if packet.data_flags == JUDPPacketDataFlags.SINGLE_PACKET:
			self.complete_packets.add(packet.sequence_number)
		elif packet.data_flags == JUDPPacketDataFlags.FIRST_PACKET:
			self.start_packets.add(packet.sequence_number)
	
	def _get_complete_message_sequence(self, start):
		"""Return the full sequence of packets pointed at by start
		if it makes a complete message, None otherwise.
		
		Invariant assumed: Message is multi-packet.
		"""
		packets = self.packets_received
		
		assert start in packets and packets[start].data_flags == JUDPPacketDataFlags.FIRST_PACKET
		
		sequence_number = start
		
		while (sequence_number in packets
				and (packets[sequence_number].data_flags
					!= JUDPPacketDataFlags.LAST_PACKET)):
			if sequence_number != start:
				assert packets[sequence_number].data_flags == JUDPPacketDataFlags.NORMAL_PACKET
			sequence_number += 1
		
		# sequence is only completed if the loop terminated on the last
		# packet inclusive
		if sequence_number not in packets:
			return None
		
		return [packets[num] for num in range(start, sequence_number+1)]
	
	def pop_messages(self):
		messages = []
		packets = self.packets_received
		
		packets_to_remove = []
		
		for complete in self.complete_packets:
			packet = packets[complete]
			packets_to_remove.append(packet)
			messages.append(_messages.Message([packet]))
		
		for start in self.start_packets:
			completed = self._get_complete_message_sequence(start)
			if completed is not None:
				packets_to_remove.extend(completed)
				messages.append(_messages.Message(completed))
		
		for packet in packets_to_remove:
			self.complete_packets.discard(packet.sequence_number)
			self.start_packets.discard(packet.sequence_number)
			del packets[packet.sequence_number]
		
		return messages

class MessageBatcher(object):
	pass

class UnsupportedJUDPVersion(Exception):
	def __init__(self, actual_version, expected_version):
		self.actual_version = actual_version
		self.expected_version = expected_version
	def __str__(self):
		return (
			"Expected JUDP version to be {expected_version} but was "
			"{actual_version} instead."
			.format(
				actual_version=self.actual_version,
				expected_version=self.expected_version))

class WrongDestination(Exception):
	"""Packet not meant for us."""

class Transport(object):
	def __init__(self, component):
		self.component = component
		self.message_reassemblers = {}

	def receive(self, judp):
		if judp.transport_version == 2:
			for packet in judp.packets:
				try:
					self._receive_packet(packet)
				except WrongDestination:
					print ('Wrong packet destination: {}'
						.format(packet.destination_id))
		else:
			raise UnsupportedJUDPVersion(judp.transport_version, 2)
	
	def _get_reassembler(self, source_id):
		return self.message_reassemblers.setdefault(
			source_id, MessageReassembler())
	
	def _receive_packet(self, packet):
		if packet.destination_id != self.component.id:
			raise WrongDestination

		reassembler = self._get_reassembler(packet.source_id)
		reassembler.add_packet(packet)
	
	def pop_new_messages(self):
		return [
			message
			for reassembler in self.message_reassemblers.values()
			for message in reassembler.pop_messages()
		]
