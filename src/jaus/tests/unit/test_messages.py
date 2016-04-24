import bitstring
import pytest

import jaus.messages as messages

def test__parse_QueryHeartbeatPulse():
    data = bitstring.BitString(hex='0x0222')
    instance = messages.MessageSpecification.instantiate(data)
    assert instance == messages.QueryHeartbeatPulseMessage()
