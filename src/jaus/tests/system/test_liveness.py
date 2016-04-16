import bitstring
import pytest


from jaus.service import Component, Transport, Liveness

@pytest.mark.asyncio
def test_liveness():
    component = Component(id=0x01010101, services=[Transport, Liveness])

    data = bitstring.BitString(bin=(
        '00000010'
        '000000'
        '00' '00010010 00000000' # no HC, data size=16
        '01 00 00 00' # flags: normal priority, no ack required
        '00000001 00000001 00000001 00000001' # dest id
        '00000010 00000010 00000010 00000010' # source id
        '00000010 00100010' # message code
        '00000000 00000000' # sequence number
      ))

    component.transport.protocol.datagram_received(data.bytes, ('9.9.9.9', 6666))
    component.transoirt.protocol.close()
