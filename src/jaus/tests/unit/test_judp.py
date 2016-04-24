import asyncio
import bitstring
import collections
import pytest

from unittest.mock import Mock, patch, call, sentinel

import jaus.judp as judp


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
    def message_received_signal(self, monkeypatch):
        return Mock(name='message_received_signal')
    @pytest.fixture
    def tp(selfconnection, message_received_signal):
        return judp.JUDPProtocol(
            message_received_signal=message_received_signal,
            own_id=sentinel.own_id)

    @patch.object(judp.JUDPProtocol, 'message_received')
    def test__instance_received__unsupported_transport_version(self, message_received, tp, caplog):
        tp.instance_received(
            judp.JUDPPayload(transport_version=3, packets=[]),
            sentinel.sender)

        assert message_received.call_args_list == []
        assert 'incorrect transport version' in caplog.text

    @patch.object(judp.JUDPProtocol, 'message_received')
    def test__instance_received(self, message_received, tp, connection, caplog):
        packet1 = Mock(
            name='packet1',
            destination_id=tp.own_id,
            source_id=sentinel.source_id1)
        packet2 = Mock(
            name='packet2',
            destination_id=sentinel.other_id,
            source_id=sentinel.source_id2)
        packet3 = Mock(
            name='packet3',
            destination_id=tp.own_id,
            source_id=sentinel.source_id3)
        payload = judp.JUDPPayload(
            transport_version=2,
            packets=[
                packet1,
                packet2,
                packet3,
            ])
        connection_instance = connection.return_value
        connection_instance.receive_packet.return_value = [
            sentinel.message1,
            sentinel.message2,
        ]

        tp.instance_received(payload, sentinel.sender)

        assert tp.connection_map == {
            packet1.source_id: connection_instance,
            packet3.source_id: connection_instance,
        }

        assert connection.call_args_list == [
            call(address=sentinel.sender, protocol=tp),
            call(address=sentinel.sender, protocol=tp),
        ]

        assert message_received.call_args_list == [
            call(sentinel.message1, packet1.source_id),
            call(sentinel.message2, packet1.source_id),
            call(sentinel.message1, packet3.source_id),
            call(sentinel.message2, packet3.source_id),
        ]

        assert 'Ignoring packet sent to sentinel.other_id' in caplog.text

    @pytest.mark.asyncio
    def test__send_message(self, tp, connection):
        connection_instance = connection.return_value
        @asyncio.coroutine
        def do_nothing():
            pass
        connection_instance.send.return_value = do_nothing()

        with pytest.raises(ValueError):
            yield from tp.send_message(sentinel.unknown_id)

        tp.connection_map.add_connection(
            sentinel.known_id, sentinel.address)

        yield from tp.send_message(sentinel.known_id, foo='bar')

        assert connection_instance.send.call_args_list == [
            call(
                foo='bar',
                destination_id=sentinel.known_id,
                source_id=sentinel.own_id)
        ]

class TestJUDPPayloadSpecification:
    def test__correct_packet_serialization(self):
        packet = judp.JUDPPacket(
            message_type=0,
            data_size=16,
            data_flags=judp.JUDPPacketDataFlags.SINGLE_PACKET,
            destination_id=0x01010101,
            source_id=0x10101010,
            contents=b'\x20\x30',
            sequence_number=2)
        serialized = judp.JUDPPacketSpecification.serialize(packet)
        assert serialized == bitstring.BitString(hex=(
            '0x00' '1000' '50' '01010101' '10101010' '2030' '0200'
        ))

        deserialized = judp.JUDPPacketSpecification.instantiate(serialized)
        assert deserialized == packet
