import pytest


from jaus.service import Component, Transport, Liveness, Events
from jaus.run import install_component

import jaus.messages as messages

@pytest.mark.asyncio
def test_liveness(connection):
    conn = connection

    yield from conn.send_message(
        messages.QueryHeartbeatPulseMessage())

    result = yield from conn.receive_messages()
    assert result == [
        messages.ReportHeartbeatPulseMessage()]
