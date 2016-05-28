import pytest
import asyncio
import collections
import bitstring
import inspect
import logging

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
    def connection_lost(self, exc):
        logging.debug('Testing connection lost {}'.format(exc))

class JUDPConnection:
    def __init__(self, protocol, remote_addr, component_id,
            destination_id):
        self.remote_addr = remote_addr
        self.protocol = protocol
        self.sequence_number = 0
        self.id = component_id
        self.destination_id = destination_id
        self.recv_queue = asyncio.Queue()

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
        logging.info('Test transport {} sending {} to {}/{}'
            .format(
                self.id,
                message,
                self.remote_addr,
                self.destination_id))
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

    def get_existing_messages(self, message_count, types=None):
        queue = self.recv_queue
        keep, reject = [], []
        while not queue.empty() and len(keep) < message_count:
            msg = queue.get_nowait()
            if types is None or msg.message_code in types:
                keep.append(msg)
            else:
                reject.append(msg)
        for msg in reject:
            queue.put_nowait(msg)
        return keep

    @asyncio.coroutine
    def receive_messages(self, message_count=1, types=None, timeout=5):
        mr = judp.MessageReassembler()
        messages = self.get_existing_messages(message_count, types=types)

        while len(messages) < message_count:
            payload = yield from self.receive_payload(timeout)
            assert payload.transport_version == 2
            for packet in payload.packets:
                assert packet.destination_id == self.id
                if packet.ack_nack in (
                        judp.JUDPPacketACKNACKFlags.ACK,
                        judp.JUDPPacketACKNACKFlags.NACK):
                    continue
                mr.add_packet(packet)
            new_messages = mr.pop_messages()
            logging.info(
                "Test transport {} received {} from {}/{}"
                .format(
                    self.id,
                    new_messages,
                    self.remote_addr,
                    packet.source_id))
            if types is not None:
                for msg in new_messages:
                    if msg.message_code in types:
                        messages.append(msg)
                    else:
                        self.recv_queue.put_nowait(msg)
            else:
                messages += new_messages
        return messages

@asyncio.coroutine
def connect_test_udp(event_loop, port, destination_id, test_id):
    protocol = UDPProtocol()
    local_addr = ('127.0.0.1', port)
    remote_addr = ('127.0.0.1', 3794)
    yield from event_loop.create_datagram_endpoint(
        lambda: protocol,
        local_addr=local_addr)
    return JUDPConnection(
        protocol,
        remote_addr,
        test_id,
        destination_id)

slow = pytest.mark.skipif(
    not pytest.config.getoption("--runslow"),
    reason="need --runslow option to run")
