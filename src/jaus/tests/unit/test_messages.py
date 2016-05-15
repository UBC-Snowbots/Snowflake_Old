import bitstring
import pytest

import jaus.format as fmt
import jaus.messages as messages

def test__parse_QueryHeartbeatPulse():
    data = bitstring.BitString(hex='0x0222')
    instance = messages.Message._instantiate(data)
    assert instance == messages.QueryHeartbeatPulseMessage()

def test__scaledfloat__io():
    sf = fmt.specification('sf', [
        messages.ScaledFloat('foo', bytes=1, real_lower_limit=0, real_upper_limit=255)
    ])
    inst = sf(foo=5)
    assert sf._instantiate(bitstring.BitString(hex='0x05')) == inst
    assert inst._serialize() == bitstring.BitString(hex='0x05')
