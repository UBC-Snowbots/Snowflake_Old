import pytest

import jaus.messages as messages

@pytest.mark.asyncio(forbid_global_loop=True)
def test_liveness(connection):
    conn = connection

    yield from conn.send_message(
        messages.QueryHeartbeatPulseMessage())

    result = yield from conn.receive_messages()
    assert result == [
        messages.ReportHeartbeatPulseMessage()]
