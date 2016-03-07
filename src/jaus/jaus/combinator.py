from .transport import Packet

class Message(object):
	def __init__(self, source_id, contents, priority):
		self.source_id = source_id
		self.contents = contents
		self.priority = priority

def split_into_runs(seq):
	last = None
	run = []
	for i in seq:
		if last and i == last+1:
			run.append(i)
		else:
			if run:
				yield run
			run = [i]
		last = i
	if run:
		yield run

def extract_packet_sequences(parts):
	sequence_so_far = None
	for part in parts:
		data_flags = part.header.data_flags
		if data_flags == Packet.ONLY_PART:
			sequence_so_far = None
			yield [part]
		elif data_flags == Packet.FIRST_PART:
			sequence_so_far = [part]
		elif data_flags == Packet.MIDDLE_PART:
			if sequence_so_far:
				sequence_so_far.append(part)
		elif data_flags == Packet.LAST_PART:
			if sequence_so_far:
				sequence_so_far.append(part)
				yield sequence_so_far
			sequence_so_far = None

def merge_packet_sequence(parts):
	if parts:
		first = parts[0]
		return Message(
			source_id=first.header.source_id,
			contents=reduce(
				lambda acc, contents: acc + contents,
				(part.contents for part in parts),
				bytearray()),
			priority=first.header.priority)
	else:
		raise ValueError('Part sequence merge passed empty list')

class PacketAlreadyAddedError(Exception):
	"""Packet already added to Assembler."""

def group_packets_by_sequence(packets_by_seq_num):
	seq_num_runs = split_into_runs(sorted(packets_by_seq_num))
	packet_runs = (
		[
			packets_by_seq_num[num]
			for num in run]
		for run in seq_num_runs)
	print packet_runs
	groups = []
	for run in packet_runs:
		for group in extract_packet_sequences(run):
			groups.append(group)
	return groups


class Assembler(object):
	def __init__(self):
		self._unclaimed = {}
	def add_packet(self, part):
		if part.sequence_number in self._unclaimed:
			raise PacketAlreadyAddedError(part.sequence_number)
		self._unclaimed[part.sequence_number] = part
	def pop_complete_messages(self):
		packets = self._unclaimed
		packets_grouped = group_packets_by_sequence(packets)
		for seq in packets_grouped:
			for packet in seq:
				del packets[packet.sequence_number]
		messages = [
			merge_packet_sequence(group)
			for group in packets_grouped]
		return messages

class Combinator(object):
	def __init__(self):
		self._assemblers = {}
	def add_packets(self, packets):
		for packet in packets:
			source_id = part.header.source_id
			assembler = self._assemblers.get(source_id)
			if assembler is None:
				assembler = Assembler(source_id)
				self._assemblers[source_id] = assembler
			assembler.add_packet(packet)
	def pop_messages(self):
		results = []
		for assembler in self._assemblers.itervalues():
			results.append(assembler.pop_complete_messages())
		return results
