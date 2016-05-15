import asyncio
import pytest

import util

from jaus.service import Component, Transport, Events, Liveness
from jaus.messages import Id

from jaus.run import install_component

@pytest.fixture
def connection(event_loop, unused_tcp_port, request):
    component = Component(
        id=Id(subsystem=0x0101, node=0x01, component=0x01),
        services=[Transport, Events, Liveness])

    event_loop.run_until_complete(install_component(component))

    f = asyncio.async(util.connect_test_udp(
        event_loop,
        unused_tcp_port,
        Id(subsystem=0x0101, node=0x01, component=0x01)))
    event_loop.run_until_complete(f)
    conn = f.result()

    def finalizer():
        component.transport.protocol.close()
    request.addfinalizer(finalizer)
    return conn
