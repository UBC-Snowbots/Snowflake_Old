import asyncio
import collections
import bitstring
import inspect
import pytest


from jaus.service import Component, Transport, Liveness
from jaus.run import install_component

import jaus.judp as judp
import jaus.messages as messages


class UDPProtocol:
    def __init__(self):
        self.recv_queue = asyncio.Queue()
    def connection_made(self, transport):
        self.transport = transport
    def datagram_received(self, data, address):
        self.recv_queue.put_nowait((data, address))
    @asyncio.coroutine
    def receive(self):
        result = yield from self.recv_queue.get()
        return result
    @asyncio.coroutine
    def send(self, data, destination):
        self.transport.sendto(data, destination)

class JUDPConnection:
    def __init__(self, protocol, remote_addr, component_id,
            destination_id):
        self.remote_addr = remote_addr
        self.protocol = protocol
        self.sequence_number = 0
        self.id = component_id
        self.destination_id = destination_id

    @asyncio.coroutine
    def send_payload(self, payload):
        data = payload._serialize_to_bytes()
        yield from self.protocol.send(data, self.remote_addr)

    @asyncio.coroutine
    def receive_payload(self, timeout=5):
        data, addr = yield from asyncio.wait_for(
            self.protocol.receive(),
            timeout=timeout)

        assert addr == self.remote_addr
        payload = judp.JUDPPayload._instantiate(
            bitstring.ConstBitStream(data))
        return payload

    @asyncio.coroutine
    def send_message(self, message, **kwargs):
        packets = judp.make_packets(
            message,
            self.sequence_number,
            destination_id=self.destination_id,
            source_id=self.id,
            **kwargs)
        self.sequence_number += len(packets)
        payloads = judp.split_payloads(packets)
        results = yield from asyncio.gather(*(
            self.send_payload(payload)
            for payload in payloads))
        return results

    @asyncio.coroutine
    def receive_messages(self, payloads=1, timeout=5):
        mr = judp.MessageReassembler()
        payload_count = 0

        while payload_count < payloads:
            payload = yield from self.receive_payload(timeout)
            payload_count += 1
            assert payload.transport_version == 2
            for packet in payload.packets:
                assert hex(packet.destination_id) == hex(self.id)
                if packet.ack_nack in (
                        judp.JUDPPacketACKNACKFlags.ACK,
                        judp.JUDPPacketACKNACKFlags.NACK):
                    continue
                mr.add_packet(packet)
        return mr.pop_messages()

@asyncio.coroutine
def connect_test_udp(event_loop, port, destination_id):
    protocol = UDPProtocol()
    local_addr = ('127.0.0.1', port)
    remote_addr = ('127.0.0.1', 3794)
    yield from event_loop.create_datagram_endpoint(
        lambda: protocol,
        local_addr=local_addr)
    return JUDPConnection(
        protocol,
        remote_addr,
        0x10101010,
        destination_id)

@pytest.mark.asyncio
def test_liveness(event_loop, unused_tcp_port):
    component = Component(
        id=0x01010101,
        services=[Transport, Liveness])

    yield from install_component(component)

    conn = yield from connect_test_udp(
        event_loop, unused_tcp_port, 0x01010101)

    yield from conn.send_message(
        messages.QueryHeartbeatPulseMessage())

    result = yield from conn.receive_messages()
    assert result == [
        messages.ReportHeartbeatPulseMessage()]
