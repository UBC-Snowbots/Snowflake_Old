import asyncio
import pytest

import util

from jaus.service import (
    Component,
    Transport,
    Events,
    AccessControl,
    Management,
    Liveness,
    Discovery,
    ListManager,
    LocalPoseSensor,
    VelocityStateSensor,
    LocalWaypointDriver,
    LocalWaypointListDriver,
    PrimitiveDriver)
from jaus.messages import Id
from jaus.judp import JUDPProtocol
import jaus.messages as messages

from jaus.run import install_protocol


@pytest.fixture
def test_id():
    return messages.Id(subsystem=0x1010, node=0x10, component=0x10)

@pytest.fixture
def test_id2():
    return messages.Id(subsystem=0x1011, node=0x11, component=0x11)

@pytest.fixture
def event_loop(event_loop, request):
    real_close = event_loop.close
    event_loop.close = lambda:None
    def finalizer():
        event_loop.run_until_complete(asyncio.sleep(0))
        real_close()
    request.addfinalizer(finalizer)
    return event_loop

@pytest.fixture
def protocol(event_loop, request):
    protocol = JUDPProtocol(loop=event_loop)
    event_loop.run_until_complete(install_protocol(protocol, loop=event_loop))
    def finalizer():
        protocol.close()
    request.addfinalizer(finalizer)
    return protocol

@pytest.fixture
def component_core(protocol, event_loop, request):
    component = Component(
        id=Id(subsystem=0x0101, node=0x01, component=0x01),
        name='Core',
        node_name='JAUS',
        subsystem_name='Snowbots',
        protocol=protocol,
        loop=event_loop,
        services=[
            Transport,
            Events,
            AccessControl,
            Management,
            Liveness,
            Discovery,
            ListManager])
    def finalizer():
        component.close()
    request.addfinalizer(finalizer)
    return component

@pytest.fixture
def component_navigation(protocol, event_loop, request):
    component = Component(
        id=Id(subsystem=0x0101, node=0x01, component=0x02),
        name='Mobility',
        node_name='JAUS',
        subsystem_name='Snowbots',
        protocol=protocol,
        loop=event_loop,
        services=[
            Transport,
            Events,
            AccessControl,
            Management,
            Liveness,
            Discovery,
            ListManager,
            LocalPoseSensor,
            VelocityStateSensor,
            LocalWaypointDriver,
            LocalWaypointListDriver,
            PrimitiveDriver])
    def finalizer():
        component.close()
    request.addfinalizer(finalizer)
    return component

def make_connection(event_loop, unused_tcp_port, test_id, destination_id=Id(subsystem=0x0101, node=0x01, component=0x01)):
    event_loop.set_debug(True)
    f = asyncio.async(
        util.connect_test_udp(
            event_loop,
            unused_tcp_port,
            destination_id,
            test_id),
        loop=event_loop)
    event_loop.run_until_complete(f)
    conn = f.result()
    return conn

@pytest.fixture
def connection(event_loop, unused_tcp_port, component_core, component_navigation, test_id):
    return make_connection(event_loop, unused_tcp_port, test_id)

@pytest.fixture
def connection_multicast(event_loop, unused_tcp_port, component_core, component_navigation, test_id):
    return make_connection(event_loop, unused_tcp_port, test_id, destination_id=Id(65535, 255, 255))

@pytest.fixture
def unused_tcp_port2(unused_tcp_port):
    # hacky hax, may fail...
    return unused_tcp_port+1

@pytest.fixture
def connection2(event_loop, unused_tcp_port2, component_core, component_navigation, test_id2):
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

    event_loop.run_until_complete(init())

    def finalizer():
        @asyncio.coroutine
        def finalize():
            yield from connection.send_message(
                messages.ReleaseControlMessage())

            #results = yield from connection.receive_messages(types=(messages.MessageCode.RejectControl,))
            results = yield from connection.receive_messages()
            assert results == [messages.RejectControlMessage(
                response_code=messages.RejectControlResponseCode.CONTROL_RELEASED)]
        event_loop.run_until_complete(finalize())
    request.addfinalizer(finalizer)

    return connection
