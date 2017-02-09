import asyncio
import bitstring
import collections
import pytest

from unittest.mock import Mock, patch, call, sentinel

import jaus.judp as judp
import jaus.messages as messages


class TestMessageReassembler(object):
    @pytest.fixture
    def mr(self):
        return judp.MessageReassembler()

    def test_add_packet(self, mr):
        packet1 = Mock(name='packet1')
        mr.add_packet(packet1)

        assert mr.packets_received == {
          packet1.sequence_number: packet1
        }

        with pytest.raises(judp.DuplicatePacket):
            mr.add_packet(packet1)

            packet2 = Mock(name='packet2')
            packet2.data_flags = judp.JUDPPacketDataFlags.SINGLE_PACKET
            mr.add_packet(packet2)

            assert mr.complete_packets == {packet2.sequence_number}
            assert mr.packets_received == {
              packet1.sequence_number: packet1,
              packet2.sequence_number: packet2,
            }

            packet3 = Mock(name='packet3')
            packet3.data_flags = judp.JUDPPacketDataFlags.FIRST_PACKET
            mr.add_packet(packet3)
            assert mr.start_packets == {packet3.sequence_number}
            assert mr.packets_received == {
              packet1.sequence_number: packet1,
              packet2.sequence_number: packet2,
              packet3.sequence_number: packet3,
            }

    def test__get_complete_message_sequence(self, mr):
        start = Mock(name='start')
        start.data_flags = judp.JUDPPacketDataFlags.FIRST_PACKET
        middle1 = Mock(name='middle1')
        middle1.data_flags = judp.JUDPPacketDataFlags.NORMAL_PACKET
        middle2 = Mock(name='middle2')
        middle2.data_flags = judp.JUDPPacketDataFlags.NORMAL_PACKET
        end = Mock(name='end')
        end.data_flags = judp.JUDPPacketDataFlags.LAST_PACKET
        only = Mock(name='only')
        only.data_flags = judp.JUDPPacketDataFlags.SINGLE_PACKET

        mr.packets_received = {
            1: Mock(name='wrong_place'),
            # None: missing end
            3: start,
            # success
            5: start,
            6: end,
            # error: new packet starts before end
            7: start,
            8: middle1,
            # success
            9: start,
            10: middle1,
            11: end,
            # None: missing end
            12: start,
            13: middle1,
            # success
            15: start,
            16: middle1,
            17: middle2,
            18: end,
            # error: duplicate start
            # error: single packet mixed in
            19: start,
            20: start,
            21: only,
        }

        with pytest.raises(AssertionError):
            mr._get_complete_message_sequence(1)
            assert mr._get_complete_message_sequence(3) is None
        with pytest.raises(AssertionError):
            mr._get_complete_message_sequence(4)
        assert mr._get_complete_message_sequence(5) == [start, end]
        with pytest.raises(AssertionError):
            mr._get_complete_message_sequence(7)
        assert mr._get_complete_message_sequence(9) == [start, middle1, end]
        assert mr._get_complete_message_sequence(12) is None
        assert mr._get_complete_message_sequence(15) == [start, middle1, middle2, end]
        with pytest.raises(AssertionError):
            mr._get_complete_message_sequence(19)
        with pytest.raises(AssertionError):
            mr._get_complete_message_sequence(20)

    @patch('jaus.messages.assemble_message')
    def test_pop_messages(self, assemble_message, mr):
        start = Mock(name='start')
        start.data_flags = judp.JUDPPacketDataFlags.FIRST_PACKET
        start.sequence_number = 5
        end = Mock(name='end')
        end.data_flags = judp.JUDPPacketDataFlags.LAST_PACKET
        end.sequence_number = 6
        only = Mock(name='only')
        only.data_flags = judp.JUDPPacketDataFlags.SINGLE_PACKET
        only.sequence_number = 1

        mr.packets_received = {
            1: only,
            5: start,
            6: end,
        }

        mr.complete_packets = {1}
        assert mr.pop_messages() == [assemble_message.return_value]
        assert mr.complete_packets == set()
        assert mr.packets_received == {5: start, 6: end}
        assert assemble_message.call_args == call([only])

        mr.start_packets = {5}
        assert mr.pop_messages() == [assemble_message.return_value]
        assert mr.complete_packets == set()
        assert mr.start_packets == set()
        assert mr.packets_received == {}
        assert assemble_message.call_args == call([start, end])

class TestJUDPProtocol(object):
    @pytest.fixture
    def connection(self, monkeypatch):
        connection = Mock(name='Connection')
        monkeypatch.setattr(
            judp, 'Connection', connection)
        return connection
    @pytest.fixture
    def message_received(self, monkeypatch):
        @asyncio.coroutine
        def message_received(*args, **kwargs):
            pass
        return Mock(name='message_received', wraps=message_received)
    @pytest.fixture
    def own_id(self):
        return Mock(
            name='own_id',
            subsystem=sentinel.own_subsystem,
            node=sentinel.own_node,
            component=sentinel.own_component)
    @pytest.fixture
    def other_id(self):
        return Mock(
            name='other_id',
            subsystem=sentinel.other_subsystem,
            node=sentinel.other_node,
            component=sentinel.other_component)
    @pytest.fixture
    def tp(selfconnection, message_received, own_id, event_loop):
        return judp.JUDPProtocol({
            own_id: message_received
        }, loop=event_loop)

    @pytest.mark.asyncio
    def test__instance_received__unsupported_transport_version(self,
            message_received, tp, caplog):

        yield from tp.instance_received(
            judp.JUDPPayload(transport_version=3, packets=[]),
            sentinel.sender)

        assert message_received.call_args_list == []
        assert 'incorrect transport version' in caplog.text

    @pytest.mark.asyncio(forbid_global_loop=True)
    def test__instance_received(self, message_received, tp, connection, caplog, own_id, other_id, event_loop):
        packet1 = Mock(
            name='packet1',
            destination_id=own_id,
            source_id=sentinel.source_id1)
        packet2 = Mock(
            name='packet2',
            destination_id=other_id,
            source_id=sentinel.source_id2)
        packet3 = Mock(
            name='packet3',
            destination_id=own_id,
            source_id=sentinel.source_id3)
        payload = judp.JUDPPayload(
            transport_version=2,
            packets=[
                packet1,
                packet2,
                packet3,
            ])
        connection_instance = connection.return_value
        @asyncio.coroutine
        def receive_packet(*args, **kwargs):
            return [
                sentinel.message1,
                sentinel.message2,
            ]
        connection_instance.receive_packet = Mock(wraps=receive_packet)

        yield from tp.instance_received(payload, sentinel.sender)

        assert tp.connection_map_map == {
            own_id:{
                packet1.source_id: connection_instance,
                packet3.source_id: connection_instance,
            },
        }

        assert connection.call_args_list == [
            call(address=sentinel.sender, protocol=tp, loop=event_loop),
            call(address=sentinel.sender, protocol=tp, loop=event_loop),
        ]

        assert message_received.call_args_list == [
            call(message=sentinel.message1, source_id=packet1.source_id),
            call(message=sentinel.message2, source_id= packet1.source_id),
            call(message=sentinel.message1, source_id= packet3.source_id),
            call(message=sentinel.message2, source_id= packet3.source_id),
        ]

        assert 'Ignoring packet sent to {}'.format(other_id) in caplog.text

    @pytest.mark.asyncio
    def test__send_message(self, tp, connection):
        connection_instance = connection.return_value
        @asyncio.coroutine
        def do_nothing():
            pass
        connection_instance.send.return_value = do_nothing()

        with pytest.raises(ValueError):
            yield from tp.send_message(
                destination_id=sentinel.unknown_id,
                source_id=sentinel.own_id,
                message=sentinel.message)

        tp.connection_map_map[sentinel.own_id] = judp.ConnectionMap(tp)
        tp.connection_map_map[sentinel.own_id].add_connection(
            sentinel.known_id, sentinel.address)

        yield from tp.send_message(
            destination_id=sentinel.known_id,
            source_id=sentinel.own_id,
            message=sentinel.message,
            foo='bar')

        assert connection_instance.send.call_args_list == [
            call(
                foo='bar',
                message=sentinel.message,
                destination_id=sentinel.known_id,
                source_id=sentinel.own_id)
        ]

class TestJUDPPayload:
    def test__correct_packet_serialization(self):
        packet = judp.JUDPPacket(
            message_type=0,
            data_size=16,
            data_flags=judp.JUDPPacketDataFlags.SINGLE_PACKET,
            destination_id=messages.Id(
                subsystem=0x0101, node=0x01, component=0x01),
            source_id=messages.Id(
                subsystem=0x1010, node=0x10, component=0x10),
            contents=b'\x20\x30',
            sequence_number=2)
        serialized = packet._serialize()
        assert serialized == bitstring.BitString(hex=(
            '0x00' '1000' '50' '01010101' '10101010' '2030' '0200'
        ))

        deserialized = judp.JUDPPacket._instantiate(serialized)
        assert deserialized == packet
    def test__data_size(self):
        assert judp.packet_overhead(
            {
                'HC_flags': judp.JUDPPacketHCFlags.NONE,
            }) == 14
        assert judp.packet_overhead(
            {
                'HC_flags': judp.JUDPPacketHCFlags.REQUESTED,
            }) == 16

def test__split_to_chunks():
    assert judp.split_to_chunks(b'abcdefghijk', 3) == [
            b'abc',
            b'def',
            b'ghi',
            b'jk'
        ]

def judp_packet(contents, flags, seq_num,
        ack_nack=judp.JUDPPacketACKNACKFlags.NO_RESPONSE_REQUIRED):
    return judp.JUDPPacket(
        contents=contents,
        data_flags=flags,
        sequence_number=seq_num,
        destination_id=sentinel.destination_id,
        source_id=sentinel.source_id,
        ack_nack=ack_nack
    )

def packet_sequence(records):
    return [judp_packet(*record) for record in records]

@pytest.mark.parametrize('seq_num,chunks,packets', [
    (
        0,
        [b'a'],
        packet_sequence([
            (b'a', judp.JUDPPacketDataFlags.SINGLE_PACKET, 0),
        ])),
    (
        2,
        [b'a', b'b'],
        packet_sequence([
            (b'a', judp.JUDPPacketDataFlags.FIRST_PACKET, 2),
            (b'b', judp.JUDPPacketDataFlags.LAST_PACKET, 3),
        ])),
    (
        4,
        [b'a', b'b', b'c'],
        packet_sequence([
            (b'a', judp.JUDPPacketDataFlags.FIRST_PACKET, 4),
            (b'b', judp.JUDPPacketDataFlags.NORMAL_PACKET, 5),
            (b'c', judp.JUDPPacketDataFlags.LAST_PACKET, 6),
        ])),
])
@patch.object(judp, 'split_to_chunks')
@patch.object(messages, 'Message')
def test__make_packets(Message, split_to_chunks, seq_num,
        chunks, packets):
    message = Message.return_value
    message_bytes = message._serialize_to_bytes.return_value
    split_to_chunks.return_value = chunks

    assert judp.make_packets(
        message,
        seq_num,
        destination_id=sentinel.destination_id,
        source_id=sentinel.source_id) == packets

    assert split_to_chunks.call_args == call(message_bytes, judp.CHUNK_SIZE)
    assert message._serialize_to_bytes.call_args == call()

packet_half_chunk = Mock(
    name='packet_half_chunk',
    data_size=judp.CHUNK_SIZE//2 - 1)
packet_full_chunk = Mock(
    name='packet_full_chunk',
    data_size=judp.CHUNK_SIZE-1)

@pytest.mark.parametrize('packets,payload,leftover', [
    (
        [packet_half_chunk],
        judp.JUDPPayload(
            transport_version=2,
            packets=[packet_half_chunk]),
        []),
    (
        [packet_full_chunk, packet_half_chunk],
        judp.JUDPPayload(
            transport_version=2,
            packets=[packet_full_chunk]),
        [packet_half_chunk]),
    (
        [packet_half_chunk, packet_half_chunk, packet_half_chunk],
        judp.JUDPPayload(
            transport_version=2,
            packets=[packet_half_chunk, packet_half_chunk]),
        [packet_half_chunk]),
])
def test__consume_one_payload(packets, payload, leftover):
    assert judp.consume_one_payload(packets) == (payload, leftover)

@patch.object(judp, 'consume_one_payload')
def test__split_payloads(consume_one_payload):
    consume_one_payload.side_effect = [
        (sentinel.p1, sentinel.r1),
        (sentinel.p2, sentinel.r2),
        (sentinel.p3, False)
    ]

    assert judp.split_payloads(sentinel.packets) == [
        sentinel.p1, sentinel.p2, sentinel.p3
    ]

    assert consume_one_payload.call_args_list == [
        call(sentinel.packets),
        call(sentinel.r1),
        call(sentinel.r2),
    ]

@pytest.fixture
def protocol():
    return Mock(name='protocol')

@pytest.fixture
def connection(protocol, event_loop, monkeypatch):
    monkeypatch.setattr(
        judp,
        'MessageReassembler',
        Mock(name='MessageReassembler'))
    return judp.Connection(sentinel.address, protocol, loop=event_loop)

@pytest.mark.asyncio(forbid_global_loop=True)
def test__connection__send(connection, monkeypatch):
    @asyncio.coroutine
    def send_packet_fn(packet):
        pass

    send_packet = Mock(wraps=send_packet_fn)
    monkeypatch.setattr(judp.Connection, 'send_packet', send_packet)
    make_packets = Mock(name='make_packets')
    monkeypatch.setattr(judp, 'make_packets', make_packets)

    make_packets.return_value = [
        sentinel.packet1,
        sentinel.packet2
    ]

    try:
        yield from connection.send(sentinel.message)
        assert send_packet.call_args_list == [
            call(sentinel.packet1), call(sentinel.packet2)
        ]
        assert make_packets.call_args == call(
            sentinel.message,
            sequence_number=0)
    finally:
        connection.close()

@pytest.mark.asyncio(forbid_global_loop=True)
def test__connection__send_packet__without_ack(connection):
    anf = judp.JUDPPacketACKNACKFlags
    connection.close()

    packet = Mock(
        name='packet',
        ack_nack=anf.NO_RESPONSE_REQUIRED)
    yield from connection.send_packet(packet)
    assert connection.packets_to_send == [packet]

@pytest.mark.asyncio(forbid_global_loop=True)
def test__connection__send_packet__with_ack(connection, protocol, event_loop):
    anf = judp.JUDPPacketACKNACKFlags
    connection.close()

    @asyncio.coroutine
    def mock_dispatcher():
        for response in responses:
            while packet.sequence_number not in connection.ack_nack:
                yield from asyncio.sleep(0)
            connection.ack_nack[packet.sequence_number].set_result(
                response)
            del connection.ack_nack[packet.sequence_number]

    packet = Mock(
        name='packet',
        ack_nack=anf.RESPONSE_REQUIRED)

    responses = [anf.NACK]*3 + [anf.ACK]
    yield from asyncio.gather(
        connection.send_packet(packet),
        mock_dispatcher(), loop=event_loop)
    assert connection.packets_to_send == [packet]*4

    with pytest.raises(judp.SendError) as e:
        responses = [anf.NACK]*5
        yield from asyncio.gather(
            connection.send_packet(packet),
            mock_dispatcher(), loop=event_loop)
    assert e.value.packet is packet

@pytest.mark.asyncio(forbid_global_loop=True)
def test__connection__receive_packet__ack_nack(connection, event_loop):
    anf = judp.JUDPPacketACKNACKFlags
    connection.close()

    packet = Mock(
        name='packet',
        ack_nack=anf.ACK)
    recv = asyncio.Future(loop=event_loop)
    connection.ack_nack[packet.sequence_number] = recv
    result = yield from connection.receive_packet(packet)
    assert result == []
    assert packet.sequence_number not in connection.ack_nack
    assert recv.result() is anf.ACK

@pytest.mark.asyncio(forbid_global_loop=True)
def test__connection__receive_packet__no_response_required(connection):
    anf = judp.JUDPPacketACKNACKFlags
    connection.close()
    message_reassembler = connection.message_reassembler

    packet = Mock(
        name='packet',
        ack_nack=anf.NO_RESPONSE_REQUIRED)
    result = yield from connection.receive_packet(packet)
    assert result == message_reassembler.pop_messages.return_value
    assert message_reassembler.add_packet.call_args == call(packet)

@pytest.mark.asyncio(forbid_global_loop=True)
def test__connection__receive_packet__response_required(connection, monkeypatch):
    anf = judp.JUDPPacketACKNACKFlags
    connection.close()
    message_reassembler = connection.message_reassembler

    @asyncio.coroutine
    def send_packet_fn(*args, **kwargs):
        pass
    send_packet = Mock(wraps=send_packet_fn)
    monkeypatch.setattr(judp.Connection, 'send_packet', send_packet)

    packet = judp_packet(
        b'abc',
        judp.JUDPPacketDataFlags.SINGLE_PACKET,
        0,
        anf.RESPONSE_REQUIRED)

    result = yield from connection.receive_packet(packet)
    assert send_packet.call_args == call(judp_packet(
        b'',
        judp.JUDPPacketDataFlags.SINGLE_PACKET,
        0,
        anf.ACK))
    assert result == message_reassembler.pop_messages.return_value
    assert message_reassembler.add_packet.call_args == call(packet)

@pytest.mark.asyncio(forbid_global_loop=True)
def test__connection___send_batching_loop(connection, protocol, monkeypatch, event_loop):
    connection.close()
    split_payloads = Mock(name='split_payloads')
    monkeypatch.setattr(judp, 'split_payloads', split_payloads)

    @asyncio.coroutine
    def test():
        packets_to_send = [sentinel.a, sentinel.b]
        connection.packets_to_send = packets_to_send
        payload1 = Mock(name='payload1')
        payload2 = Mock(name='payload2')
        split_payloads.return_value = [payload1, payload2]

        yield from asyncio.sleep(0.5, loop=event_loop)
        assert protocol.transport.sendto.call_args_list == [
            call(payload1._serialize_to_bytes.return_value, connection.address),
            call(payload2._serialize_to_bytes.return_value, connection.address),
        ]
        assert split_payloads.call_args == call(packets_to_send)

        send_batching_loop.cancel()

    send_batching_loop = asyncio.async(
        connection._send_batching_loop(), loop=event_loop)
    cancelled, result = yield from asyncio.gather(
        send_batching_loop,
        test(),
        return_exceptions=True,
        loop=event_loop)

    assert isinstance(cancelled, asyncio.CancelledError)
    if result is not None:
        raise result

@patch.object(judp, 'Connection')
def test__connectionmap__add_connection(Connection, event_loop):
    protocol = sentinel.protocol
    connection = Connection.return_value
    cm = judp.ConnectionMap(protocol, loop=event_loop)

    assert cm.add_connection(
        sentinel.source_id,
        sentinel.address) is connection
    assert Connection.call_args == call(
        address=sentinel.address,
        protocol=protocol,
        loop=event_loop)

    assert cm[sentinel.source_id] is connection
