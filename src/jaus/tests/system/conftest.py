import asyncio
import pytest

import util

from jaus.service import (
    Component,
    Transport,
    Events,
    AccessControl,
    Management,
    Liveness)
from jaus.messages import Id
import jaus.messages as messages

from jaus.run import install_component


@pytest.fixture
def test_id():
    return messages.Id(subsystem=0x1010, node=0x10, component=0x10)

@pytest.fixture
def test_id2():
    return messages.Id(subsystem=0x1011, node=0x11, component=0x11)

@pytest.fixture
def component(event_loop, request):
    component = Component(
        id=Id(subsystem=0x0101, node=0x01, component=0x01),
        services=[
            Transport,
            Events,
            AccessControl,
            Management,
            Liveness])
    event_loop.run_until_complete(install_component(component))
    def finalizer():
        component.close()
    request.addfinalizer(finalizer)
    return component

def make_connection(event_loop, unused_tcp_port, test_id):
    f = asyncio.async(util.connect_test_udp(
        event_loop,
        unused_tcp_port,
        Id(subsystem=0x0101, node=0x01, component=0x01),
        test_id))
    event_loop.run_until_complete(f)
    conn = f.result()
    return conn

@pytest.fixture
def connection(event_loop, unused_tcp_port, component, test_id):
    return make_connection(event_loop, unused_tcp_port, test_id)

@pytest.fixture
def unused_tcp_port2(unused_tcp_port):
    # hacky hax, may fail...
    return unused_tcp_port+1

@pytest.fixture
def connection2(event_loop, unused_tcp_port2, component, test_id2):
    return make_connection(event_loop, unused_tcp_port2, test_id2)

@pytest.fixture
def control_connection(connection, event_loop, request):
    @asyncio.coroutine
    def init():
        yield from connection.send_message(
            messages.RequestControlMessage(authority_code=5))

        results = yield from connection.receive_messages(types=(messages.MessageCode.ConfirmControl,))
        assert results == [messages.ConfirmControlMessage(
            response_code=messages.ConfirmControlResponseCode.CONTROL_ACCEPTED)]

    event_loop.run_until_complete(asyncio.async(init()))

    def finalizer():
        @asyncio.coroutine
        def finalize():
            yield from connection.send_message(
                messages.ReleaseControlMessage())

            #results = yield from connection.receive_messages(types=(messages.MessageCode.RejectControl,))
            results = yield from connection.receive_messages()
            assert results == [messages.RejectControlMessage(
                response_code=messages.RejectControlResponseCode.CONTROL_RELEASED)]
        event_loop.run_until_complete(asyncio.async(finalize()))
    request.addfinalizer(finalizer)

    return connection
