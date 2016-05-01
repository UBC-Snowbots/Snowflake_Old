import abc as _abc
import asyncio as _asyncio
import bitstring as _bitstring
import collections as _collections
import enum as _enum
import logging as _logging

import jaus.format as _format
import jaus.messages as _messages

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
    NACK = 2
    ACK = 3

def packet_overhead(attrs):
    if attrs['HC_flags'] != JUDPPacketHCFlags.NONE:
        return 16
    else:
        return 14

JUDPPacket = _format.specification(
    'JUDPPacket',
    specs=[
        _format.Int('message_type', bits=6),
        # if message type is not 0, bail now
        # as this is not a protocol we know
        _format.Optional(lambda attrs: attrs['message_type']==0, [
            _format.Enum('HC_flags', enum=JUDPPacketHCFlags, bits=2),
            _format.Int('data_size', bytes=2, endianness='le'),
            # these fields only exist if header compression is used
            _format.Optional(lambda attrs: attrs['HC_flags'] != JUDPPacketHCFlags.NONE, [
                _format.Int('HC_number', bytes=1),
                _format.Int('HC_length', bytes=1),
              ]),
            _format.Enum('priority', enum=JUDPPacketPriority, bits=2),
            _format.Enum('broadcast', enum=JUDPPacketBroadcastFlags, bits=2),
            _format.Enum('ack_nack', enum=JUDPPacketACKNACKFlags, bits=2),
            _format.Enum('data_flags', enum=JUDPPacketDataFlags, bits=2),
            _format.Int('destination_id', bytes=4, endianness='le'),
            _format.Int('source_id', bytes=4, endianness='le'),
            _format.Bytes(
              'contents',
              length=lambda attrs: (
                attrs['data_size']
                - packet_overhead(attrs))),
            _format.Int('sequence_number', bytes=2, endianness='le'),
          ]),
    ],
    defaults={
        'message_type': 0,
        'HC_flags': JUDPPacketHCFlags.NONE,
        'HC_number': None,
        'HC_length': None,
        'priority': JUDPPacketPriority.STANDARD,
        'broadcast': JUDPPacketBroadcastFlags.LOCAL,
        'ack_nack': JUDPPacketACKNACKFlags.NO_RESPONSE_REQUIRED,
        'data_size': lambda attrs: (
            len(attrs['contents'])
            + packet_overhead(attrs))
    })

JUDPPayload = _format.specification('JUDPPayload', specs=[
    _format.Int('transport_version', bytes=1),
    # We only support transport version 2 right now...
    _format.Optional(lambda attrs: attrs['transport_version'] == 2, [
        _format.Consume('packets', JUDPPacket),
      ]),
  ])

CHUNK_SIZE = 512

class DuplicatePacket(Exception):
    """A second packet with the same sequence number was added."""

class MessageReassembler:
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
            messages.append(
              _messages.assemble_message([packet]))

        for start in self.start_packets:
            completed = self._get_complete_message_sequence(start)
            if completed is not None:
                packets_to_remove.extend(completed)
                messages.append(
                  _messages.assemble_message(completed))

        for packet in packets_to_remove:
            self.complete_packets.discard(packet.sequence_number)
            self.start_packets.discard(packet.sequence_number)
            del packets[packet.sequence_number]

        return messages

def split_to_chunks(data, size):
    return [data[pos:pos+size] for pos in range(0, len(data), size)]

def make_packets(message, sequence_number, **kwargs):
    """Split the message into packets, passing on extra arguments to the packet constructor."""
    data = message._serialize_to_bytes()
    chunks = split_to_chunks(data, CHUNK_SIZE)
    sequence_numbers = (
        num for num, chunk in enumerate(chunks, start=sequence_number))
    if len(chunks) == 1:
        data_flags = [JUDPPacketDataFlags.SINGLE_PACKET]
    elif len(chunks) >= 2:
        data_flags = (
            [JUDPPacketDataFlags.FIRST_PACKET] +
            [JUDPPacketDataFlags.NORMAL_PACKET] * (len(chunks) - 2) +
            [JUDPPacketDataFlags.LAST_PACKET])
    return [
        JUDPPacket(
            data_flags=flags,
            contents=chunk,
            sequence_number=sequence_number,
            **kwargs)
        for flags, chunk, sequence_number
        in zip(data_flags, chunks, sequence_numbers)]

def consume_one_payload(packets):
    """Pack as many packets as possible into a payload, returning
      it as well as the remaining packets."""
    payload_size = 1
    packets_consumed = []
    while payload_size < CHUNK_SIZE and packets:
        packet = packets[0]
        assert packet.data_size < CHUNK_SIZE
        if payload_size + packet.data_size > CHUNK_SIZE:
            break
        packets_consumed.append(packet)
        packets = packets[1:]
        payload_size += packet.data_size
    return JUDPPayload(
        transport_version=2,
        packets=packets_consumed), packets

def split_payloads(packets):
    payloads = []
    while packets:
        payload, packets = consume_one_payload(packets)
        payloads.append(payload)
    return payloads

class SendError(Exception):
    """Failed to send packet."""
    def __init__(self, packet):
        super().__init__(
            'Failed to send packet {}'
            .format(packet))
        self.packet = packet

class Connection:
    def __init__(self, address, protocol):
        self.protocol = protocol
        self.address = address
        self.message_reassembler = MessageReassembler()
        self.ack_nack = {}
        self.packets_to_send = []
        self.sequence_number = 0

        self.batching_loop = _asyncio.async(
            self._send_batching_loop())

    @_asyncio.coroutine
    def _send_batching_loop(self):
        while True:
            if self.packets_to_send:
                payloads = split_payloads(self.packets_to_send)
                self.packets_to_send = []
                for payload in payloads:
                    self.protocol.transport.sendto(
                        payload._serialize_to_bytes(),
                        self.address)
            yield from _asyncio.sleep(0.5)

    @_asyncio.coroutine
    def send(self, message, **kwargs):
        packets = make_packets(
            message,
            sequence_number=self.sequence_number,
            **kwargs)
        self.sequence_number += len(packets)
        yield from _asyncio.gather(
            *[self.send_packet(packet) for packet in packets])

    @_asyncio.coroutine
    def send_packet(self, packet):
        self.packets_to_send.append(packet)

        if packet.ack_nack is JUDPPacketACKNACKFlags.RESPONSE_REQUIRED:
            assert packet.sequence_number not in self.ack_nack
            tries = 0
            while tries < 5:
                sent = _asyncio.Future()
                self.ack_nack[packet.sequence_number] = sent
                result = yield from sent
                if result is JUDPPacketACKNACKFlags.ACK:
                    return
                tries += 1
                self.packets_to_send.append(packet)
            raise SendError(packet)

    @_asyncio.coroutine
    def receive_packet(self, packet):
        if packet.ack_nack in (JUDPPacketACKNACKFlags.ACK, JUDPPacketACKNACKFlags.NACK):
            self.ack_nack[packet.sequence_number].set_result(packet.ack_nack)
            del self.ack_nack[packet.sequence_number]
            return []
        else:
            if packet.ack_nack is JUDPPacketACKNACKFlags.RESPONSE_REQUIRED:
                yield from self.send_packet(packet._replace(
                    contents=b'',
                    data_size=packet.data_size-len(packet.contents),
                    ack_nack=JUDPPacketACKNACKFlags.ACK))
            self.message_reassembler.add_packet(packet)
            return self.message_reassembler.pop_messages()

    def close(self):
        self.batching_loop.cancel()

class ConnectionMap(_collections.UserDict):
    def __init__(self, protocol, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.protocol = protocol
    def add_connection(self, source_id, address):
        if source_id not in self.data:
            self.data[source_id] = Connection(
                address=address,
                protocol=self.protocol)
        return self.data[source_id]

class FormatSpecificationProtocol(metaclass=_abc.ABCMeta):
    def __init__(self, specification):
        self.specification = specification
    def datagram_received(self, data, address):
        instance = self.specification._instantiate(
            _bitstring.ConstBitStream(bytes=data))
        _asyncio.async(self.instance_received(instance, address))
    @_abc.abstractmethod
    def instance_received(self, instance, address):
        pass

class JUDPProtocol(FormatSpecificationProtocol):
    def __init__(self, own_id, message_received_signal):
        super().__init__(specification=JUDPPayload)
        self.connection_map = ConnectionMap(self)
        self.message_received_signal = message_received_signal
        self.own_id = own_id
    def connection_made(self, transport):
        self.transport = transport
    @_asyncio.coroutine
    def instance_received(self, instance, address):
        payload = instance

        if payload.transport_version == 2:
            for packet in payload.packets:
                source_id = packet.source_id
                if packet.destination_id != self.own_id:
                    _logging.info(
                        'Ignoring packet sent to %s, we are %s',
                        str(packet.destination_id),
                        str(self.own_id))
                    continue
                connection = self.connection_map.add_connection(
                    source_id=source_id,
                    address=address)
                messages = yield from connection.receive_packet(packet)
                for message in messages:
                    self.message_received(message, source_id)
        else:
            _logging.warning(
                'Received payload with incorrect'
                ' transport version from %s: %d',
                str(address), payload.transport_version)

    def message_received(self, message, source_id):
        responses = self.message_received_signal.send(
            message_code=message.message_code, message=message, source_id=source_id)
        for response in responses:
            _asyncio.async(response)

    @_asyncio.coroutine
    def send_message(self, destination_id, *args, **kwargs):
        try:
            connection = self.connection_map[destination_id]
        except KeyError:
            raise ValueError(
                'No record of {}'.format(destination_id))
        yield from connection.send(
            *args,
            destination_id=destination_id,
            source_id=self.own_id,
            **kwargs)

    def connection_lost(self, exc):
        for connection in self.connection_map.iteritems():
            connection.close()
