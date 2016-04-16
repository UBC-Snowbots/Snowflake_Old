import collections
import pytest

from unittest.mock import Mock, patch, call, sentinel

from jaus.judp import (
    DuplicatePacket,
    MessageReassembler,
    JUDPPacketDataFlags,
    JUDPPacketACKNACKFlags,
    JUDPProtocol,
)


class TestMessageReassembler(object):
    @pytest.fixture
    def mr(self):
        return MessageReassembler()

    def test_add_packet(self, mr):
        packet1 = Mock(name='packet1')
        mr.add_packet(packet1)

        assert mr.packets_received == {
          packet1.sequence_number: packet1
        }

        with pytest.raises(DuplicatePacket):
            mr.add_packet(packet1)

            packet2 = Mock(name='packet2')
            packet2.data_flags = JUDPPacketDataFlags.SINGLE_PACKET
            mr.add_packet(packet2)

            assert mr.complete_packets == {packet2.sequence_number}
            assert mr.packets_received == {
              packet1.sequence_number: packet1,
              packet2.sequence_number: packet2,
            }

            packet3 = Mock(name='packet3')
            packet3.data_flags = JUDPPacketDataFlags.FIRST_PACKET
            mr.add_packet(packet3)
            assert mr.start_packets == {packet3.sequence_number}
            assert mr.packets_received == {
              packet1.sequence_number: packet1,
              packet2.sequence_number: packet2,
              packet3.sequence_number: packet3,
            }

    def test__get_complete_message_sequence(self, mr):
        start = Mock(name='start')
        start.data_flags = JUDPPacketDataFlags.FIRST_PACKET
        middle1 = Mock(name='middle1')
        middle1.data_flags = JUDPPacketDataFlags.NORMAL_PACKET
        middle2 = Mock(name='middle2')
        middle2.data_flags = JUDPPacketDataFlags.NORMAL_PACKET
        end = Mock(name='end')
        end.data_flags = JUDPPacketDataFlags.LAST_PACKET
        only = Mock(name='only')
        only.data_flags = JUDPPacketDataFlags.SINGLE_PACKET

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
        start.data_flags = JUDPPacketDataFlags.FIRST_PACKET
        start.sequence_number = 5
        end = Mock(name='end')
        end.data_flags = JUDPPacketDataFlags.LAST_PACKET
        end.sequence_number = 6
        only = Mock(name='only')
        only.data_flags = JUDPPacketDataFlags.SINGLE_PACKET
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
    def component(self):
        return Mock(name='component')
    @pytest.fixture
    def tp(self, component):
        return Transport(component, own_id=sentinel.own_id)

    def test_receive_unsupported_transport_version(self, tp):
        judp = Mock(name='judp')
        judp.transport_version = sentinel.unsupported

        with pytest.raises(UnsupportedJUDPVersion):
            tp._receive_payload(judp, sentinel.sender)

    @patch('jaus.transport.Transport._receive_packet')
    def test_receive(self, _receive_packet, tp):
        packet1 = Mock(name='packet1')
        packet2 = Mock(name='packet2')
        packet3 = Mock(name='packet3')
        judp = Mock(name='judp')
        judp.transport_version = 2
        judp.packets = [
            packet1,
            packet2,
            packet3,
        ]

        _receive_packet.side_effect = [
            None,
            WrongDestination,
            None
        ]

        tp._receive_payload(judp, sentinel.sender)

        assert _receive_packet.call_args_list == [
            call(packet1, sentinel.sender),
            call(packet2, sentinel.sender),
            call(packet3, sentinel.sender),
        ]

    """@patch('jaus.transport.MessageReassembler')
    def test__get_reassembler(self, MessageReassembler, tp):
        assert tp._get_reassembler(sentinel.source_id) == MessageReassembler.return_value

        tp.message_reassemblers = {
            sentinel.source_id: sentinel.existing_reassembler
        }

        assert tp._get_reassembler(sentinel.source_id) == sentinel.existing_reassembler"""

    def test__receive_packet__wrong_destination(self, tp, component):
        packet = Mock(name='packet')

        with pytest.raises(WrongDestination):
            tp._receive_packet(packet, sentinel.sender)

    @patch('jaus.transport.Transport._get_connection')
    def test__receive_packet(self, _get_connection, tp, component):
        connection = _get_connection.return_value
        packet = Mock(name='packet')
        packet.destination_id = component.id
        packet.ack_nack = JUDPPacketACKNACKFlags.ACK

        tp._receive_packet(packet, sentinel.sender)
        assert _get_connection.call_args == call(
            packet.source_id, sentinel.sender)
        assert connection.receive_acknack.call_args == call(packet)

        packet.ack_nack = JUDPPacketACKNACKFlags.RESPONSE_REQUIRED

        tp._receive_packet(packet, sentinel.sender)
        assert _get_connection.call_args == call(
            packet.source_id, sentinel.sender)
        assert connection.receive_packet.call_args == call(packet)

    def test__pop_new_messages(self, tp):
        connection1 = Mock(name='connection1')
        connection2 = Mock(name='connection2')

        connection1.pop_messages.return_value = [
            sentinel.msg1,
            sentinel.msg2,
        ]
        connection2.pop_messages.return_value = [
            sentinel.msg3,
            sentinel.msg4,
        ]

        tp.connections = collections.OrderedDict([
                (sentinel.sender1, connection1),
                (sentinel.sender2, connection2),
            ])

        assert tp._pop_new_messages() == [
            (sentinel.sender1, sentinel.msg1),
            (sentinel.sender1, sentinel.msg2),
            (sentinel.sender2, sentinel.msg3),
            (sentinel.sender2, sentinel.msg4),
        ]
