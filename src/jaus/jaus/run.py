import asyncio as _asyncio
import logging as _logging

import jaus.messages as _messages
import jaus.service as _service
import jaus.judp as _judp

@_asyncio.coroutine
def install_protocol(protocol, multicast_addr='239.255.0.1', loop=None):
    if loop is None:
        loop = _asyncio.get_event_loop()
    import struct, socket
    group = '239.255.0.1'
    addrinfo = socket.getaddrinfo(group, None)[0]

    # Create a socket
    s = socket.socket(addrinfo[0], socket.SOCK_DGRAM)

    # Allow multiple copies of this program on one machine
    # (not strictly needed)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind it to the port
    s.bind(('0.0.0.0', 3794))

    group_bin = socket.inet_pton(addrinfo[0], addrinfo[4][0])
    _logging.info('Addinfo: {}'.format(addrinfo))
    ttl_bin = struct.pack('@i', 32)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl_bin)
    s.setsockopt(socket.SOL_SOCKET, socket.IP_MULTICAST_LOOP, 1)
    # Join group
    mreq = group_bin + struct.pack('=I', socket.INADDR_ANY)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    transport, protocol = yield from loop.create_datagram_endpoint(
        lambda: protocol,
        sock=s)

def run_event_loop(components, protocol, loop=None):
    if loop is None:
        loop = _asyncio.get_event_loop()
    try:
        loop.run_forever()
    finally:
        for component in components:
            component.close()
        protocol.close()
        loop.close()

def run():
    subsystem = 104
    protocol = _judp.JUDPProtocol()
    core_component = _service.Component(
        id=_messages.Id(subsystem=0x0101, node=0x01, component=0x01),
        name='Core',
        node_name='JAUS',
        subsystem_name='Snowbots',
        protocol=protocol,
        services=[
            _service.Transport,
            _service.Events,
            _service.AccessControl,
            _service.Management,
            _service.Liveness,
            _service.Discovery,
            _service.ListManager])
    mobility_component = _service.Component(
        id=_messages.Id(subsystem=subsystem, node=0x01, component=0x02),
        name='Mobility',
        node_name='JAUS',
        subsystem_name='Snowbots',
        protocol=protocol,
        services=[
            _service.Transport,
            _service.Events,
            _service.AccessControl,
            _service.Management,
            _service.Liveness,
            _service.Discovery,
            _service.ListManager,
            _service.LocalPoseSensor,
            _service.VelocityStateSensor,
            _service.LocalWaypointDriver,
            _service.LocalWaypointListDriver,
            _service.PrimitiveDriver])
    _asyncio.get_event_loop().run_until_complete(
        install_protocol(protocol))
    run_event_loop(
        components=(core_component, mobility_component),
        protocol=protocol)

if __name__ == '__main__':
    _logging.basicConfig(level='DEBUG')
    run()
