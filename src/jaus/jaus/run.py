import asyncio as _asyncio


@_asyncio.coroutine
def install_component(component, loop=None):
    if loop is None:
        loop = _asyncio.get_event_loop()
    yield from loop.create_datagram_endpoint(
        lambda: component.transport.protocol,
        local_addr=('127.0.0.1', 3794))

def run_event_loop(component, loop=None):
    if loop is None:
        loop = _asyncio.get_event_loop()
    try:
        loop.run_forever()
    finally:
        component.transport.protocol.close()
        loop.close()
